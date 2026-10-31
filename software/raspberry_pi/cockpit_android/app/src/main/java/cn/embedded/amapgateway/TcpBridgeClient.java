package cn.embedded.amapgateway;

import android.util.Log;

import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.SocketTimeoutException;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;

/** Byte-transparent TCP leg between an Android emulator and the host's USB-TTL bridge. */
final class TcpBridgeClient {
    interface LineListener {
        void onLine(String line);
        void onConnectionChanged(boolean connected);
    }

    private static final String TAG = "CockpitTcpBridge";
    private final String host;
    private final int port;
    private final LineListener listener;
    private final Object writeLock = new Object();
    private final ArrayBlockingQueue<byte[]> pending = new ArrayBlockingQueue<>(256);
    private volatile boolean running;
    private volatile Socket socket;
    private volatile OutputStream output;
    private Thread worker;

    TcpBridgeClient(String endpoint, LineListener listener) {
        if (!endpoint.startsWith("tcp://")) throw new IllegalArgumentException("Not a TCP endpoint");
        String address = endpoint.substring(6);
        int colon = address.lastIndexOf(':');
        if (colon <= 0 || colon == address.length() - 1) throw new IllegalArgumentException("Use tcp://HOST:PORT");
        host = address.substring(0, colon);
        port = Integer.parseInt(address.substring(colon + 1));
        if (port < 1 || port > 65535) throw new IllegalArgumentException("Invalid TCP port");
        this.listener = listener;
    }

    void start() {
        running = true;
        worker = new Thread(this::runLoop, "emulator-serial-bridge");
        worker.setDaemon(true);
        worker.start();
    }

    boolean send(byte[] bytes) {
        // BroadcastReceiver callbacks run on Android's main thread. Enqueue
        // here; only the dedicated writer thread may perform socket I/O.
        return output != null && pending.offer(Arrays.copyOf(bytes, bytes.length));
    }

    void stop() {
        running = false;
        closeSocket();
        if (worker != null) worker.interrupt();
    }

    private void closeSocket() {
        synchronized (writeLock) {
            output = null;
            Socket old = socket;
            socket = null;
            if (old != null) try { old.close(); } catch (Exception ignored) { }
            pending.clear();
            if (old != null) listener.onConnectionChanged(false);
        }
    }

    private void runLoop() {
        while (running) {
            try {
                Socket connected = new Socket();
                connected.connect(new InetSocketAddress(host, port), 2000);
                connected.setSoTimeout(1000);
                connected.setTcpNoDelay(true);
                synchronized (writeLock) {
                    socket = connected;
                    output = connected.getOutputStream();
                }
                Thread writer = new Thread(() -> writeLoop(connected), "emulator-serial-writer");
                writer.setDaemon(true);
                writer.start();
                listener.onConnectionChanged(true);
                Log.i(TAG, "Connected to host UART bridge at " + host + ":" + port);
                InputStream input = connected.getInputStream();
                ByteArrayOutputStream line = new ByteArrayOutputStream();
                byte[] chunk = new byte[2048];
                while (running) {
                    int count;
                    try { count = input.read(chunk); }
                    catch (SocketTimeoutException timeout) { continue; }
                    if (count < 0) break;
                    for (int i = 0; i < count; i++) {
                        int value = chunk[i] & 0xFF;
                        if (value == '\n') {
                            String message = new String(line.toByteArray(), StandardCharsets.UTF_8).trim();
                            line.reset();
                            if (!message.isEmpty()) listener.onLine(message);
                        } else if (value != '\r') {
                            if (line.size() < 65536) line.write(value);
                            else line.reset();
                        }
                    }
                }
            } catch (Exception error) {
                if (running) Log.w(TAG, "Host UART bridge unavailable; retrying", error);
            } finally {
                closeSocket();
            }
            if (running) try { Thread.sleep(1000); }
            catch (InterruptedException interrupted) { Thread.currentThread().interrupt(); return; }
        }
    }

    private void writeLoop(Socket connected) {
        try {
            OutputStream stream = connected.getOutputStream();
            while (running && socket == connected) {
                byte[] bytes = pending.poll(500, TimeUnit.MILLISECONDS);
                if (bytes == null) continue;
                stream.write(bytes);
                stream.flush();
            }
        } catch (InterruptedException interrupted) {
            Thread.currentThread().interrupt();
        } catch (Exception error) {
            if (running) Log.w(TAG, "Bridge write failed", error);
            closeSocket();
        }
    }
}
