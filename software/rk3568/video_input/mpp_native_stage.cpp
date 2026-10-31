// Native GStreamer appsink probe. It does not cross a Unix pipe or call Python
// per frame. publish_direct imports the MPP DMA-BUF into RGA without copying
// NV12; publish_direct_cached experiments with a cached BGR output DMA heap.
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <climits>
#include <cstdint>
#include <csignal>
#include <ctime>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <string>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <gst/app/gstappsink.h>
#include <gst/allocators/gstdmabuf.h>
#include <gst/video/video.h>
#include "im2d.h"

namespace {
using Clock = std::chrono::steady_clock;
std::atomic<bool> stop_requested{false};

void request_stop(int) { stop_requested.store(true); }

struct SharedHeader {
    char magic[8];
    uint32_t width;
    uint32_t height;
    uint64_t sequence;
    uint64_t mono_ns;
};
static_assert(sizeof(SharedHeader) == 32, "shared header layout changed");

struct Buffer {
    int fd = -1;
    void *ptr = MAP_FAILED;
    size_t size = 0;
    rga_buffer_handle_t handle = 0;
    Buffer() = default;
    Buffer(const Buffer &) = delete;
    Buffer &operator=(const Buffer &) = delete;
    ~Buffer() {
        if (handle) releasebuffer_handle(handle);
        if (ptr != MAP_FAILED) munmap(ptr, size);
        if (fd >= 0) close(fd);
    }
};

bool allocate(Buffer &buffer, int heap_fd, size_t size) {
    dma_heap_allocation_data request{};
    request.len = size;
    request.fd_flags = O_RDWR | O_CLOEXEC;
    if (ioctl(heap_fd, DMA_HEAP_IOCTL_ALLOC, &request) != 0) return false;
    buffer.fd = static_cast<int>(request.fd);
    buffer.size = size;
    buffer.ptr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, buffer.fd, 0);
    if (buffer.ptr == MAP_FAILED) return false;
    buffer.handle = importbuffer_fd(buffer.fd, static_cast<int>(size));
    return buffer.handle != 0;
}

bool sync_buffer(int fd, unsigned int flags) {
    dma_buf_sync request{};
    request.flags = flags;
    return ioctl(fd, DMA_BUF_IOCTL_SYNC, &request) == 0;
}

double ms(Clock::time_point a, Clock::time_point b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
}

double cpu_ms(clockid_t clock_id) {
    timespec value{};
    if (clock_gettime(clock_id, &value) != 0) return 0;
    return static_cast<double>(value.tv_sec) * 1000 + value.tv_nsec / 1000000.0;
}

bool copy_nv12(GstSample *sample, const GstVideoInfo &info, Buffer &input, int width, int height) {
    GstVideoFrame frame{};
    if (!gst_video_frame_map(&frame, &info, gst_sample_get_buffer(sample), GST_MAP_READ)) {
        std::fprintf(stderr, "gst_video_frame_map failed\n");
        return false;
    }
    const auto *y = static_cast<const unsigned char *>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 0));
    const auto *uv = static_cast<const unsigned char *>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 1));
    const int y_stride = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
    const int uv_stride = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 1);
    const bool valid = y && uv && y_stride >= width && uv_stride >= width;
    bool ok = valid && sync_buffer(input.fd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE);
    if (ok) {
        auto *target = static_cast<unsigned char *>(input.ptr);
        for (int row = 0; row < height; ++row)
            std::memcpy(target + static_cast<size_t>(row) * width, y + static_cast<size_t>(row) * y_stride, width);
        auto *target_uv = target + static_cast<size_t>(width) * height;
        for (int row = 0; row < height / 2; ++row)
            std::memcpy(target_uv + static_cast<size_t>(row) * width,
                        uv + static_cast<size_t>(row) * uv_stride, width);
        ok = sync_buffer(input.fd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE);
    }
    gst_video_frame_unmap(&frame);
    if (!valid) std::fprintf(stderr, "invalid NV12 plane layout\n");
    if (!ok) std::fprintf(stderr, "DMA-BUF CPU write sync failed: %s\n", std::strerror(errno));
    return ok;
}

bool import_nv12_dmabuf(GstSample *sample, const GstVideoInfo &info,
                        int width, int height, rga_buffer_handle_t &handle,
                        rga_buffer_t &source, std::string &reason) {
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    if (gst_buffer_n_memory(buffer) != 1) {
        reason = "NV12 is not one DMA-BUF memory block";
        return false;
    }
    GstMemory *memory = gst_buffer_peek_memory(buffer, 0);
    if (!gst_is_dmabuf_memory(memory)) {
        reason = "MPP output is not DMA-BUF";
        return false;
    }
    gsize memory_offset = 0, max_size = 0;
    gst_memory_get_sizes(memory, &memory_offset, &max_size);
    GstVideoMeta *meta = gst_buffer_get_video_meta(buffer);
    const int y_stride = meta ? meta->stride[0] : GST_VIDEO_INFO_PLANE_STRIDE(&info, 0);
    const int uv_stride = meta ? meta->stride[1] : GST_VIDEO_INFO_PLANE_STRIDE(&info, 1);
    const gsize y_offset = meta ? meta->offset[0] : GST_VIDEO_INFO_PLANE_OFFSET(&info, 0);
    const gsize uv_offset = meta ? meta->offset[1] : GST_VIDEO_INFO_PLANE_OFFSET(&info, 1);
    if (memory_offset != 0 || y_offset != 0 || y_stride < width || uv_stride != y_stride ||
        y_stride % 16 || uv_offset % static_cast<gsize>(y_stride) ||
        uv_offset < static_cast<gsize>(y_stride) * height ||
        max_size > INT_MAX ||
        max_size < uv_offset + static_cast<gsize>(uv_stride) * (height / 2)) {
        reason = "unsupported DMA-BUF stride or offset layout";
        std::fprintf(stderr, "DMA layout: offset=%zu max=%zu y=%d uv=%d y_offset=%zu uv_offset=%zu\n",
                     static_cast<size_t>(memory_offset), static_cast<size_t>(max_size),
                     y_stride, uv_stride, static_cast<size_t>(y_offset),
                     static_cast<size_t>(uv_offset));
        return false;
    }
    const int hstride = static_cast<int>(uv_offset / y_stride);
    if (hstride % 2) {
        reason = "DMA-BUF height stride is odd";
        return false;
    }
    const int fd = gst_dmabuf_memory_get_fd(memory);
    if (fd < 0) {
        reason = "DMA-BUF has no usable fd";
        return false;
    }
    // The sample stays referenced until the synchronous RGA call finishes.
    handle = importbuffer_fd(fd, static_cast<int>(max_size));
    if (!handle) {
        reason = "RGA cannot import MPP DMA-BUF";
        return false;
    }
    source = wrapbuffer_handle(handle, width, height, RK_FORMAT_YCbCr_420_SP,
                               y_stride, hstride);
    imsetColorSpace(&source, IM_YUV_BT601_FULL_RANGE);
    return true;
}
}  // namespace

int main(int argc, char **argv) {
    // librga on this board writes its version to stdout. Reserve stdout for
    // the final machine-readable result and redirect library text to stderr.
    const int report_fd = dup(STDOUT_FILENO);
    if (report_fd < 0 || dup2(STDERR_FILENO, STDOUT_FILENO) < 0) return 2;
    if (argc != 4 && argc != 5) {
        std::fprintf(stderr, "usage: mpp_native_stage MODE URL SECONDS [SHM_PATH]\n");
        return 2;
    }
    const std::string mode = argv[1];
    const std::string url = argv[2];
    const double duration = std::strtod(argv[3], nullptr);
    const bool cached_output = mode == "publish_direct_cached";
    const bool publishing = mode == "publish" || mode == "publish_direct" || cached_output;
    const bool direct = mode == "publish_direct" || cached_output;
    if ((mode != "nv12" && mode != "cpu_bgr" && mode != "rga_bgr" && !publishing) ||
        (publishing != (argc == 5)) ||
        url.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789:/._?=-") != std::string::npos ||
        duration < 0 || duration > 86400 || (!publishing && duration < 5)) {
        std::fprintf(stderr, "invalid mode, URL or duration\n");
        return 2;
    }
    std::signal(SIGTERM, request_stop);
    std::signal(SIGINT, request_stop);
    gst_init(&argc, &argv);
    const std::string pipeline_text =
        "souphttpsrc location=" + url + " ! multipartdemux ! jpegparse ! mppjpegdec ! "
        "video/x-raw,format=NV12,colorimetry=1:4:0:0 ! " +
        (mode == "cpu_bgr" ? "videoconvert ! video/x-raw,format=BGR ! " : "") +
        "appsink name=out sync=false max-buffers=1 drop=true";
    GError *parse_error = nullptr;
    GstElement *pipeline = gst_parse_launch(pipeline_text.c_str(), &parse_error);
    if (!pipeline || parse_error) {
        std::fprintf(stderr, "pipeline parse: %s\n", parse_error ? parse_error->message : "unknown");
        if (parse_error) g_error_free(parse_error);
        if (pipeline) gst_object_unref(pipeline);
        return 3;
    }
    GstElement *sink = gst_bin_get_by_name(GST_BIN(pipeline), "out");
    if (!sink) {
        std::fprintf(stderr, "appsink missing\n");
        gst_object_unref(pipeline);
        return 3;
    }
    GstBus *bus = gst_element_get_bus(pipeline);
    Buffer input, output;
    int shared_fd = -1;
    unsigned char *shared = nullptr;
    size_t shared_size = 0;
    rga_buffer_t source{}, target{};
    int width = 0, height = 0;
    bool first_memory_dmabuf = false;
    unsigned int first_n_memory = 0;
    double copy_total = 0, import_total = 0, rga_total = 0, publish_total = 0;
    double publish_sync_start_total = 0, publish_copy_total = 0, publish_sync_end_total = 0;
    double import_cpu_total = 0, rga_submit_cpu_total = 0, publish_cpu_total = 0;
    double publish_sync_start_cpu_total = 0, publish_copy_cpu_total = 0, publish_sync_end_cpu_total = 0;
    unsigned long frames = 0;
    std::string error;
    auto start = Clock::now();
    const double process_cpu_start = cpu_ms(CLOCK_PROCESS_CPUTIME_ID);
    const double main_cpu_start = cpu_ms(CLOCK_THREAD_CPUTIME_ID);
    auto first = start;
    if (gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
        error = "pipeline failed to start";
    while (error.empty() && !stop_requested.load() &&
           (duration == 0 || ms(start, Clock::now()) < duration * 1000)) {
        GstSample *sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sink), 200 * GST_MSECOND);
        if (!sample) {
            GstMessage *message = gst_bus_pop_filtered(bus, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
            if (message) {
                if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) {
                    GError *gst_error = nullptr;
                    gchar *detail = nullptr;
                    gst_message_parse_error(message, &gst_error, &detail);
                    error = gst_error ? gst_error->message : "GStreamer error";
                    if (gst_error) g_error_free(gst_error);
                    g_free(detail);
                } else {
                    error = "source reached EOS";
                }
                gst_message_unref(message);
            }
            continue;
        }
        GstVideoInfo info{};
        if (!gst_video_info_from_caps(&info, gst_sample_get_caps(sample))) {
            error = "invalid sample caps";
            gst_sample_unref(sample);
            break;
        }
        const int sample_width = GST_VIDEO_INFO_WIDTH(&info);
        const int sample_height = GST_VIDEO_INFO_HEIGHT(&info);
        if (frames == 0) {
            width = sample_width;
            height = sample_height;
            first = Clock::now();
            GstBuffer *buffer = gst_sample_get_buffer(sample);
            first_n_memory = gst_buffer_n_memory(buffer);
            first_memory_dmabuf = first_n_memory > 0 &&
                gst_is_dmabuf_memory(gst_buffer_peek_memory(buffer, 0));
            if (mode == "rga_bgr" || publishing) {
                if (width % 16 || height % 2) {
                    error = "RGA probe needs width divisible by 16 and even height";
                } else {
                    const char *heap_path = cached_output ? "/dev/dma_heap/system" : "/dev/dma_heap/system-uncached";
                    const int heap = open(heap_path, O_RDWR | O_CLOEXEC);
                    if (heap < 0) {
                        error = "cannot open DMA heap";
                    } else {
                        const bool allocated = (direct || allocate(input, heap, static_cast<size_t>(width) * height * 3 / 2)) &&
                                               allocate(output, heap, static_cast<size_t>(width) * height * 3);
                        close(heap);
                        if (!allocated) {
                            error = "DMA heap allocation or RGA import failed";
                        } else {
                            if (!direct)
                                source = wrapbuffer_handle(input.handle, width, height, RK_FORMAT_YCbCr_420_SP);
                            target = wrapbuffer_handle(output.handle, width, height, RK_FORMAT_BGR_888);
                            if (!direct) imsetColorSpace(&source, IM_YUV_BT601_FULL_RANGE);
                            imsetColorSpace(&target, IM_RGB_FULL);
                            if (!direct && imcheck(source, target, {}, {}) != IM_STATUS_NOERROR)
                                error = "RGA imcheck failed";
                            if (error.empty() && publishing) {
                                shared_size = sizeof(SharedHeader) + static_cast<size_t>(width) * height * 3;
                                shared_fd = open(argv[4], O_RDWR | O_CLOEXEC | O_NOFOLLOW);
                                struct stat shared_stat{};
                                if (shared_fd < 0 || fstat(shared_fd, &shared_stat) != 0 ||
                                    !S_ISREG(shared_stat.st_mode) ||
                                    static_cast<size_t>(shared_stat.st_size) != shared_size) {
                                    error = "invalid shared output file";
                                } else {
                                    void *mapped = mmap(nullptr, shared_size, PROT_READ | PROT_WRITE,
                                                        MAP_SHARED, shared_fd, 0);
                                    if (mapped == MAP_FAILED) error = "cannot map shared output file";
                                    else shared = static_cast<unsigned char *>(mapped);
                                }
                            }
                        }
                    }
                }
            }
        } else if (width != sample_width || height != sample_height) {
            error = "source geometry changed";
        }
        if (error.empty() && (mode == "rga_bgr" || publishing)) {
            const auto before_copy = Clock::now();
            const double before_import_cpu = cpu_ms(CLOCK_THREAD_CPUTIME_ID);
            rga_buffer_handle_t direct_handle = 0;
            if (direct) {
                import_nv12_dmabuf(sample, info, width, height, direct_handle, source, error);
            } else if (!copy_nv12(sample, info, input, width, height)) {
                error = "NV12 map/copy failed";
            }
            if (error.empty() && direct && imcheck(source, target, {}, {}) != IM_STATUS_NOERROR) {
                error = "RGA cannot convert imported MPP DMA-BUF";
            }
            if (error.empty()) {
                const auto after_copy = Clock::now();
                const double after_import_cpu = cpu_ms(CLOCK_THREAD_CPUTIME_ID);
                const IM_STATUS status = imcvtcolor(source, target, RK_FORMAT_YCbCr_420_SP,
                                                     RK_FORMAT_BGR_888, IM_YUV_TO_RGB_BT601_FULL);
                if (status != IM_STATUS_SUCCESS) {
                    error = std::string("RGA imcvtcolor: ") + imStrError(status);
                } else {
                    const auto after_rga = Clock::now();
                    const double after_rga_cpu = cpu_ms(CLOCK_THREAD_CPUTIME_ID);
                    if (direct) import_total += ms(before_copy, after_copy);
                    else copy_total += ms(before_copy, after_copy);
                    import_cpu_total += after_import_cpu - before_import_cpu;
                    rga_total += ms(after_copy, after_rga);
                    rga_submit_cpu_total += after_rga_cpu - after_import_cpu;
                    if (publishing) {
                        const auto before_publish = Clock::now();
                        const double before_publish_cpu = cpu_ms(CLOCK_THREAD_CPUTIME_ID);
                        const size_t bgr_size = static_cast<size_t>(width) * height * 3;
                        const auto before_sync_start = Clock::now();
                        const double before_sync_start_cpu = cpu_ms(CLOCK_THREAD_CPUTIME_ID);
                        const bool sync_started = sync_buffer(output.fd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ);
                        publish_sync_start_total += ms(before_sync_start, Clock::now());
                        publish_sync_start_cpu_total += cpu_ms(CLOCK_THREAD_CPUTIME_ID) - before_sync_start_cpu;
                        if (!sync_started) {
                            error = "output DMA-BUF read sync failed";
                        } else if (flock(shared_fd, LOCK_EX) != 0) {
                            error = "shared output lock failed";
                        } else {
                            const auto before_publish_copy = Clock::now();
                            const double before_publish_copy_cpu = cpu_ms(CLOCK_THREAD_CPUTIME_ID);
                            std::memcpy(shared + sizeof(SharedHeader), output.ptr, bgr_size);
                            publish_copy_total += ms(before_publish_copy, Clock::now());
                            publish_copy_cpu_total += cpu_ms(CLOCK_THREAD_CPUTIME_ID) - before_publish_copy_cpu;
                            SharedHeader header{};
                            std::memcpy(header.magic, "ADASMJPG", 8);
                            header.width = static_cast<uint32_t>(width);
                            header.height = static_cast<uint32_t>(height);
                            header.sequence = frames + 1;
                            header.mono_ns = static_cast<uint64_t>(
                                std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    Clock::now().time_since_epoch()).count());
                            std::memcpy(shared, &header, sizeof(header));
                            flock(shared_fd, LOCK_UN);
                        }
                        if (sync_started) {
                            const auto before_sync_end = Clock::now();
                            const double before_sync_end_cpu = cpu_ms(CLOCK_THREAD_CPUTIME_ID);
                            if (!sync_buffer(output.fd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ))
                                error = "output DMA-BUF read sync end failed";
                            publish_sync_end_total += ms(before_sync_end, Clock::now());
                            publish_sync_end_cpu_total += cpu_ms(CLOCK_THREAD_CPUTIME_ID) - before_sync_end_cpu;
                        }
                        publish_total += ms(before_publish, Clock::now());
                        publish_cpu_total += cpu_ms(CLOCK_THREAD_CPUTIME_ID) - before_publish_cpu;
                    }
                }
            }
            if (direct_handle) releasebuffer_handle(direct_handle);
        }
        if (error.empty()) ++frames;
        gst_sample_unref(sample);
    }
    const double elapsed = ms(start, Clock::now()) / 1000;
    if (shared) munmap(shared, shared_size);
    if (shared_fd >= 0) close(shared_fd);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(bus);
    gst_object_unref(sink);
    gst_object_unref(pipeline);
    const double process_cpu_total = cpu_ms(CLOCK_PROCESS_CPUTIME_ID) - process_cpu_start;
    const double main_cpu_total = cpu_ms(CLOCK_THREAD_CPUTIME_ID) - main_cpu_start;
    dprintf(report_fd,
            "{\"mode\":\"%s\",\"frames\":%lu,\"elapsed_s\":%.3f,\"fps\":%.3f,"
            "\"first_frame_ms\":%.3f,\"width\":%d,\"height\":%d,\"gst_memory_blocks\":%u,"
            "\"first_memory_dmabuf\":%s,\"copy_ms_mean\":%.3f,\"dmabuf_import_ms_mean\":%.3f,\"rga_ms_mean\":%.3f,"
            "\"output_heap\":\"%s\",\"publish_ms_mean\":%.3f,"
            "\"publish_sync_start_ms_mean\":%.3f,\"publish_copy_ms_mean\":%.3f,\"publish_sync_end_ms_mean\":%.3f,"
            "\"cpu_process_s\":%.3f,\"cpu_main_thread_s\":%.3f,"
            "\"cpu_import_s\":%.3f,\"cpu_rga_submit_s\":%.3f,\"cpu_publish_s\":%.3f,"
            "\"cpu_publish_sync_start_s\":%.3f,\"cpu_publish_copy_s\":%.3f,\"cpu_publish_sync_end_s\":%.3f,"
            "\"error\":\"%s\"}\n",
            mode.c_str(), frames, elapsed, frames / std::max(elapsed, 0.001),
            frames ? ms(start, first) : -1.0, width, height, first_n_memory,
            first_memory_dmabuf ? "true" : "false", frames ? copy_total / frames : 0,
            frames ? import_total / frames : 0, frames ? rga_total / frames : 0,
            cached_output ? "system" : "system-uncached", frames ? publish_total / frames : 0,
            frames ? publish_sync_start_total / frames : 0,
            frames ? publish_copy_total / frames : 0,
            frames ? publish_sync_end_total / frames : 0,
            process_cpu_total / 1000, main_cpu_total / 1000,
            import_cpu_total / 1000, rga_submit_cpu_total / 1000,
            publish_cpu_total / 1000,
            publish_sync_start_cpu_total / 1000, publish_copy_cpu_total / 1000,
            publish_sync_end_cpu_total / 1000,
            error.c_str());
    close(report_fd);
    return error.empty() && frames > 0 ? 0 : 4;
}
