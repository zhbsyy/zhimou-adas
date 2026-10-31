"""One reconnectable video worker, with explicit raw/processed source labels."""
import threading
import time
from urllib.request import build_opener, ProxyHandler
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import QTimer
from control_guard import SCHEMA

RAW = 'http://127.0.0.1:8081/stream'
PROCESSED = 'http://192.168.2.3:18084/stream'
PCIE = 'http://192.168.2.3:8083/stream'


def choose_url(window):
    if getattr(window.arbiter.state,'perception_source','carla')=='pcie':
        return PCIE
    session = getattr(window, 'aeb_session', None)
    ready = bool(session and time.monotonic()-session.rk_received < 1.5
                 and session.rk.get('source')=='carla'
                 and session.rk.get('carla_aeb', {}).get('schema')==SCHEMA
                 and session.rk.get('carla_aeb', {}).get('session_id')==session.arbiter.protocol.session_id
                 and session.rk.get('carla_aeb', {}).get('epoch')==session.arbiter.epoch
                 and not session.progress.stale_reason(time.monotonic()))
    return PROCESSED if ready else RAW


def start(self):
    self._video_running = True
    if getattr(self, '_video_thread', None) and self._video_thread.is_alive():
        self._video_restart.set()
        return
    self._video_restart = threading.Event()
    self._video_mailbox_lock = threading.Lock()
    self._video_latest = None
    self._video_received_at = 0.
    self._video_decoded_count = 0
    self._video_display_timer = QTimer(self)
    self._video_display_timer.timeout.connect(lambda: display_latest(self))
    self._video_display_timer.start(33)
    self._video_thread = threading.Thread(target=lambda: worker(self), daemon=True)
    self._video_thread.start()


def restart(self):
    # No overlapping old/new readers: ask the one worker to reopen its response.
    start(self)


def display_latest(self):
    # Decode/QPixmap creation stays on Qt's GUI thread. Only one newest JPEG
    # waits for display; slow resizing cannot accumulate queued frame signals.
    with self._video_mailbox_lock:
        newest,self._video_latest = self._video_latest,None
    if newest:
        pix = QPixmap()
        if pix.loadFromData(newest):
            self._video_decoded_count += 1
            self.sig_video_frame.emit(pix)
    elif time.monotonic()-self._video_received_at>1.:
        self.lbl_video_screen.setText('视频等待 / 中断（未保留旧画面）')


def worker(self):
    opener = build_opener(ProxyHandler({}))
    while self._video_running:
        url = choose_url(self)
        self._video_restart.clear()
        label = 'RK物理四路处理结果' if url==PCIE else 'RK双模型处理后CARLA' if url == PROCESSED else '原始CARLA（未经过RK）'
        try:
            with opener.open(url, timeout=1.5) as response:
                self.sig_video_status.emit(label+' | '+url)
                buffer = bytearray()
                while self._video_running and not self._video_restart.is_set() and choose_url(self) == url:
                    chunk = response.read(8192)
                    if not chunk:
                        break
                    buffer.extend(chunk)
                    newest = None
                    while True:
                        first = buffer.find(b'\xff\xd8')
                        last = buffer.find(b'\xff\xd9', first+2) if first >= 0 else -1
                        if last < 0:
                            break
                        newest = bytes(buffer[first:last+2])
                        del buffer[:last+2]
                    if len(buffer) > 8*1024*1024:
                        raise ValueError('JPEG parser buffer exceeded limit')
                    if newest:
                        with self._video_mailbox_lock:
                            self._video_latest = newest
                            self._video_received_at = time.monotonic()
        except Exception as exc:
            self.sig_video_status.emit(label+' | 视频等待: '+str(exc)[:90])
            time.sleep(.25)


def install(window_class):
    window_class._start_video_stream = start
    window_class._restart_video_stream = restart
