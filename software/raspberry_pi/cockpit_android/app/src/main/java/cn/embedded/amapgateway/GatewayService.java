package cn.embedded.amapgateway;

import android.app.Notification;
import android.app.NotificationChannel;
import android.app.NotificationManager;
import android.app.Service;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.SharedPreferences;
import android.os.Build;
import android.os.IBinder;
import android.provider.Settings;
import android.util.Log;

import java.io.RandomAccessFile;
import java.io.FileInputStream;
import java.io.ByteArrayOutputStream;
import java.io.File;
import org.json.JSONArray;
import org.json.JSONObject;

public final class GatewayService extends Service {
    static final String ACTION_START = "cn.embedded.amapgateway.START";
    static final String ACTION_STOP = "cn.embedded.amapgateway.STOP";
    static final String AMAP_ACTION = "AUTONAVI_STANDARD_BROADCAST_SEND";
    static final String SIMULATOR_GEAR = "cn.embedded.amapgateway.SIMULATOR_GEAR";
    static final String LINK_STATE = "cn.embedded.amapgateway.LINK_STATE";
    private static final String TAG = "AmapUartGateway";
    private BroadcastReceiver receiver;
    private int sequence;
    private RandomAccessFile port;
    private String openPath;
    private int openBaud;
    private Thread commandThread;
    private volatile boolean commandRunning;
    private TcpBridgeClient tcpBridge;
    private GearLinkage gearLinkage;

    private volatile Intent lastNavIntent;
    private volatile Intent lastSimulatorIntent;
    private volatile long lastRealNavBroadcastTime;
    private Thread heartbeatThread;
    private volatile boolean heartbeatRunning;

    private String raspiPort() {
        String configured = getSharedPreferences(MainActivity.PREFS, MODE_PRIVATE)
                .getString("port", "").trim();
        // Emulator apps cannot open the Windows COM device. Ignore a stale Pi
        // /dev/tty* preference if this APK was installed over an older build.
        if (isEmulator() && !configured.startsWith("tcp://")) return "tcp://10.0.2.2:8765";
        if ("/dev/ttyAMA10".equals(configured) || "/dev/ttyS3".equals(configured)) {
            configured = "/dev/ttyAMA0";
            getSharedPreferences(MainActivity.PREFS, MODE_PRIVATE).edit()
                    .putString("port", configured).apply();
        }
        if (!configured.isEmpty()) return configured;
        String[] nodes = {"/dev/ttyAMA0", "/dev/ttyAMA10", "/dev/ttyS0", "/dev/ttyS1", "/dev/ttyUSB0"};
        for (String node : nodes) if (new File(node).exists()) return node;
        return "";
    }

    private static boolean isEmulator() {
        return Build.FINGERPRINT.startsWith("generic") || Build.FINGERPRINT.contains("emulator")
                || Build.MODEL.contains("Emulator") || Build.PRODUCT.contains("sdk_gphone");
    }

    private void startHeartbeat() {
        heartbeatRunning = true;
        heartbeatThread = new Thread(() -> {
            while (heartbeatRunning) {
                try {
                    Thread.sleep(1000);
                    Intent nav = lastNavIntent;
                    // 若最近 15 秒内有真实高德导航广播，以 1Hz 节拍持续补发心跳，避免长直道广播间隔过长触发上位机误刹车
                    if (nav != null && (System.currentTimeMillis() - lastRealNavBroadcastTime < 15000)) {
                        handleNavigation(nav, true);
                    }
                } catch (InterruptedException e) {
                    break;
                } catch (Exception ignored) { }
            }
        }, "nav-heartbeat-thread");
        heartbeatThread.setDaemon(true);
        heartbeatThread.start();
    }

    @Override
    public void onCreate() {
        super.onCreate();
        createChannel();
        startForeground(7, new Notification.Builder(this, "gateway")
                .setSmallIcon(android.R.drawable.stat_sys_upload_done)
                .setContentTitle("高德串口网关运行中")
                .setContentText("监听导航广播并转发至已配置 UART")
                .setOngoing(true).build());
        gearLinkage = new GearLinkage(this);
        ensureBackgroundLaunchAllowed();
        receiver = new BroadcastReceiver() {
            @Override public void onReceive(Context context, Intent intent) {
                Log.i(TAG, "Received AMap broadcast action=" + intent.getAction());
                if (SIMULATOR_GEAR.equals(intent.getAction())) {
                    String gear = intent.getStringExtra("gear");
                    String camera = intent.getStringExtra("camera");
                    if (gearLinkage != null) gearLinkage.onGear(gear, camera);
                    getSharedPreferences(MainActivity.PREFS, MODE_PRIVATE).edit()
                            .putString("oib_gear", gear == null ? "P" : gear).apply();
                    sendSimulatorFeedback(intent);
                } else {
                    handleNavigation(intent, false);
                }
            }
        };
        IntentFilter filter = new IntentFilter(AMAP_ACTION);
        filter.addAction(SIMULATOR_GEAR);
        if (Build.VERSION.SDK_INT >= 33) registerReceiver(receiver, filter, Context.RECEIVER_EXPORTED);
        else registerReceiver(receiver, filter);
        startCommandReceiver();
        startHeartbeat();
    }

    /**
     * Android 10+ 禁止后台进程启动 Activity，倒车影像必须在后台弹出，
     * 因此需要 SYSTEM_ALERT_WINDOW（系统认可的后台启动豁免条件）。
     * 本板已具备 root（stty 配置串口同样依赖 su），此处直接自动授权；
     * 无 root 环境将回落到用户在“设置-显示在其他应用上层”中手动授权。
     */
    private void ensureBackgroundLaunchAllowed() {
        if (isEmulator()) return;
        if (Settings.canDrawOverlays(this)) return;
        try {
            Process process = new ProcessBuilder("su", "0", "sh", "-c",
                    "appops set " + getPackageName() + " SYSTEM_ALERT_WINDOW allow").start();
            int result = process.waitFor();
            if (result == 0 && Settings.canDrawOverlays(this)) {
                Log.i(TAG, "已自动授予悬浮窗权限，后台可弹出倒车影像");
            } else {
                Log.w(TAG, "自动授予悬浮窗权限未生效，请手动授权后再试倒车影像");
            }
        } catch (Exception error) {
            Log.w(TAG, "自动授予悬浮窗权限失败，请手动授权", error);
        }
    }

    private void startCommandReceiver() {
        final String path = raspiPort();
        final int baud = getSharedPreferences(MainActivity.PREFS, MODE_PRIVATE).getInt("baud", 115200);
        if (path.isEmpty()) {
            Log.w(TAG, "UART 未配置，档位报文接收线程未启动");
            return;
        }
        if (path.startsWith("tcp://")) {
            try {
                tcpBridge = new TcpBridgeClient(path, new TcpBridgeClient.LineListener() {
                    @Override public void onLine(String line) { handleControlLine(line); }
                    @Override public void onConnectionChanged(boolean connected) {
                        getSharedPreferences(MainActivity.PREFS, MODE_PRIVATE).edit()
                                .putBoolean("bridge_connected", connected).apply();
                        Intent state = new Intent(LINK_STATE);
                        state.setPackage(getPackageName());
                        state.putExtra("connected", connected);
                        sendBroadcast(state);
                        if (connected) {
                            Intent simulator = lastSimulatorIntent;
                            if (simulator != null) sendSimulatorFeedback(simulator);
                            Intent navigation = lastNavIntent;
                            if (navigation != null) handleNavigation(navigation, true);
                        }
                    }
                });
                tcpBridge.start();
                Log.i(TAG, "模拟器串口桥接线程已启动：" + path);
            } catch (IllegalArgumentException error) {
                Log.e(TAG, "模拟器桥接地址错误：" + path, error);
            }
            return;
        }
        commandRunning = true;
        commandThread = new Thread(() -> {
            try {
                // Android 设备未必提供 su；直接调用 toybox stty，失败时仍尝试打开串口。
                int result = new ProcessBuilder("/system/bin/stty", "-F", path, String.valueOf(baud),
                        "raw", "-echo", "-icanon").start().waitFor();
                if (result != 0) Log.w(TAG, "stty 配置 " + path + " 失败，仍按默认参数读取");
                try (FileInputStream input = new FileInputStream(path)) {
                    ByteArrayOutputStream lineBuffer = new ByteArrayOutputStream();
                    byte[] chunk = new byte[256];
                    while (commandRunning) {
                        int count = input.read(chunk);
                        if (count < 0) {
                            Thread.sleep(20);   // EOF 时避免空转占满 CPU
                            continue;
                        }
                        for (int i = 0; i < count; i++) {
                            int value = chunk[i] & 0xFF;
                            // 上位机挡位协议是 UTF-8 JSON + 换行；忽略同一 UART 上的二进制高德帧。
                            if (value == '\n') {
                                handleControlLine(new String(lineBuffer.toByteArray(), java.nio.charset.StandardCharsets.UTF_8).trim());
                                lineBuffer.reset();
                            } else if (value != '\r' && (value >= 0x20 || value == '\t')) {
                                if (lineBuffer.size() < 65536) lineBuffer.write(value);
                                else lineBuffer.reset();
                            }
                        }
                    }
                }
            } catch (InterruptedException interrupted) {
                Thread.currentThread().interrupt();
            } catch (Exception error) {
                Log.e(TAG, "UART command receiver stopped", error);
            }
        }, "uart-command-receiver");
        commandThread.setDaemon(true);
        commandThread.start();
        Log.i(TAG, "档位报文接收线程已启动：" + path + " @ " + baud);
    }

    /** @param line 座舱 UART 上收到的一行控制报文，形如 {"gear":"R","camera":"reverse","target_speed_kmh":30} */
    private void handleControlLine(String line) {
        if (line.isEmpty()) return;
        Log.i(TAG, "UART 控制报文：" + line);
        if (gearLinkage != null) gearLinkage.onControlLine(line);
        try {
            JSONObject value = new JSONObject(line);
            if ("ROUTE_SYNC".equals(value.optString("action"))) {
                Intent routeSync = new Intent("cn.embedded.amapgateway.ROUTE_SYNC");
                routeSync.setPackage(getPackageName());
                routeSync.putExtra("total_m", (float) value.optDouble("total_m", 0.0));
                JSONArray arr = value.optJSONArray("waypoints");
                if (arr != null) {
                    float[] flat = new float[arr.length() * 2];
                    for (int i = 0; i < arr.length(); i++) {
                        JSONArray pt = arr.getJSONArray(i);
                        flat[i * 2] = (float) pt.getDouble(0);
                        flat[i * 2 + 1] = (float) pt.getDouble(1);
                    }
                    routeSync.putExtra("flat_waypoints", flat);
                }
                sendBroadcast(routeSync);
                return;
            }

            Intent state = new Intent("cn.embedded.amapgateway.OIB_STATE");
            state.setPackage(getPackageName());
            String g = value.optString("gear", "");
            state.putExtra("gear", g);
            int spd = value.optInt("speed", -1);
            if (spd < 0) spd = value.optInt("target_speed_kmh", -1);
            state.putExtra("speed", spd);
            state.putExtra("target_speed_kmh", spd);
            int actualSpd = value.optInt("actual_speed_kmh", -1);
            if (actualSpd >= 0) state.putExtra("actual_speed_kmh", actualSpd);
            int segRemain = value.optInt("segment_remaining_distance_m", -1);
            if (segRemain >= 0) state.putExtra("segment_remaining_distance_m", segRemain);
            int routeRemain = value.optInt("route_remaining_distance_m", -1);
            if (routeRemain >= 0) state.putExtra("route_remaining_distance_m", routeRemain);
            if (value.has("route_id")) state.putExtra("route_id", value.optString("route_id"));
            if (value.has("route_name")) state.putExtra("route_name", value.optString("route_name"));
            if (value.has("simulation_running")) state.putExtra("simulation_running", value.optBoolean("simulation_running"));
            if (value.has("pos_x")) state.putExtra("pos_x", (float) value.optDouble("pos_x", 0.0));
            if (value.has("pos_y")) state.putExtra("pos_y", (float) value.optDouble("pos_y", 0.0));
            if (value.has("heading_deg")) state.putExtra("heading_deg", (float) value.optDouble("heading_deg", 0.0));
            if (value.has("pedestrian_detected")) state.putExtra("pedestrian_detected", value.optBoolean("pedestrian_detected", false));
            if (value.has("pedestrian_distance_m")) state.putExtra("pedestrian_distance_m", (float) value.optDouble("pedestrian_distance_m", 0.0));
            if (value.has("aeb_active")) state.putExtra("aeb_active", value.optBoolean("aeb_active", false));
            state.putExtra("source", "OIB");
            SharedPreferences.Editor ed = getSharedPreferences(MainActivity.PREFS, MODE_PRIVATE).edit();
            if (!g.isEmpty()) ed.putString("oib_gear", g);
            if (spd >= 0) ed.putInt("oib_speed", spd);
            ed.apply();
            sendBroadcast(state);
        } catch (Exception ignored) { }
    }

    private synchronized void handleNavigation(Intent intent, boolean isHeartbeat) {
        byte[] frame = NavFrame.fromIntent(intent, sequence++ & 0xFFFF);
        if (frame == null) {
            Log.w(TAG, "高德广播未生成 UART 帧，KEY_TYPE=" + intent.getIntExtra("KEY_TYPE", 0));
            return;
        }
        if (!isHeartbeat) {
            lastRealNavBroadcastTime = System.currentTimeMillis();
            lastNavIntent = new Intent(intent);
        }
        String path = raspiPort();
        int baud = getSharedPreferences(MainActivity.PREFS, MODE_PRIVATE).getInt("baud", 115200);
        if (path.isEmpty()) {
            Log.w(TAG, "UART 未配置；丢弃导航帧");
            return;
        }
        if (path.startsWith("tcp://")) {
            if (tcpBridge != null && tcpBridge.send(frame)) {
                Log.i(TAG, "Bridge sent keyType=10001 icon=" + intent.getIntExtra("ICON", 0)
                        + " bytes=" + frame.length);
            } else Log.w(TAG, "桥接未连接；导航帧未发送");
            return;
        }
        try {
            if (port == null || !path.equals(openPath) || baud != openBaud) {
                closePort();
                int result = new ProcessBuilder("/system/bin/stty", "-F", path, String.valueOf(baud),
                        "raw", "-echo").start().waitFor();
                if (result != 0) Log.w(TAG, "stty 配置 " + path + " 失败，继续尝试打开");
                port = new RandomAccessFile(path, "rw");
                openPath = path;
                openBaud = baud;
            }
            port.write(frame);
            Log.i(TAG, "UART sent keyType=10001 icon=" + intent.getIntExtra("ICON", 0) + " bytes=" + frame.length);
        } catch (Exception error) {
            Log.e(TAG, "UART send failed", error);
            closePort();
        }
    }

    /** 模拟器全量车辆状态回到 OIB；导航帧和状态控制共用同一双向 UART。 */
    private synchronized void sendSimulatorFeedback(Intent source) {
        lastSimulatorIntent = new Intent(source);
        String gear = source.getStringExtra("gear");
        if (gear == null || gear.trim().isEmpty()) gear = "P";
        String camera = source.getStringExtra("camera");
        int targetSpeed = source.getIntExtra("target_speed_kmh", source.getIntExtra("speed", 30));
        // Actual speed is measured telemetry; never infer it from the target speed.
        int actualSpeed = source.getIntExtra("actual_speed_kmh", 0);
        float steeringAngle = source.getFloatExtra("steering_angle", 0.0f);
        String turnSignal = source.getStringExtra("turn_signal");
        if (turnSignal == null) turnSignal = "off";
        boolean brakeLamp = source.getBooleanExtra("brake_lamp", false);
        boolean reverseLamp = source.getBooleanExtra("reverse_lamp", "R".equals(gear));
        boolean running = source.getBooleanExtra("simulation_running", true);

        String path = raspiPort();
        int baud = getSharedPreferences(MainActivity.PREFS, MODE_PRIVATE).getInt("baud", 115200);
        try {
            JSONObject state = new JSONObject();
            state.put("action", "SIMULATOR_STATE");
            state.put("gear", gear);
            state.put("camera", camera == null ? ("R".equals(gear) ? "reverse" : "none") : camera);
            state.put("target_speed_kmh", targetSpeed);
            state.put("actual_speed_kmh", actualSpeed);
            state.put("simulation_speed_kmh", targetSpeed);
            state.put("speed", targetSpeed);
            state.put("vehicle_speed", actualSpeed);
            state.put("steering_angle", (double) steeringAngle);
            state.put("turn_signal", turnSignal);
            state.put("brake_lamp", brakeLamp);
            state.put("reverse_lamp", reverseLamp);
            state.put("route_id", source.getStringExtra("route_id"));
            state.put("current_maneuver", source.getStringExtra("current_maneuver"));
            state.put("navigation_icon", source.getIntExtra("navigation_icon", 1));
            state.put("simulation_running", running);
            if (source.hasExtra("destination_x")) {
                state.put("destination_x", (double) source.getFloatExtra("destination_x", 0.0f));
                state.put("destination_y", (double) source.getFloatExtra("destination_y", 0.0f));
                state.put("destination_name", source.getStringExtra("destination_name"));
            }
            state.put("route_segment_length_m", source.getIntExtra("segment_total_distance_m", source.getIntExtra("segment_length", 0)));

            String json = state.toString() + "\n";
            byte[] bytes = json.getBytes(java.nio.charset.StandardCharsets.UTF_8);
            if (path.startsWith("tcp://")) {
                if (tcpBridge != null && tcpBridge.send(bytes))
                    Log.i(TAG, "Bridge simulator feedback=" + json.trim());
                else Log.w(TAG, "桥接未连接；座舱状态未发送");
            } else {
                if (port == null || !path.equals(openPath) || baud != openBaud) {
                    closePort();
                    new ProcessBuilder("/system/bin/stty", "-F", path, String.valueOf(baud), "raw", "-echo").start().waitFor();
                    port = new RandomAccessFile(path, "rw"); openPath = path; openBaud = baud;
                }
                port.write(bytes);
                Log.i(TAG, "UART simulator feedback=" + json.trim());
            }
        } catch (Exception error) { Log.e(TAG, "UART simulator feedback failed", error); }
    }

    private void closePort() {
        try { if (port != null) port.close(); } catch (Exception ignored) { }
        port = null;
        openPath = null;
    }

    private void createChannel() {
        if (Build.VERSION.SDK_INT >= 26) {
            NotificationManager manager = getSystemService(NotificationManager.class);
            manager.createNotificationChannel(new NotificationChannel("gateway", "导航串口网关", NotificationManager.IMPORTANCE_LOW));
        }
    }

    @Override public int onStartCommand(Intent intent, int flags, int startId) {
        if (intent != null && ACTION_STOP.equals(intent.getAction())) {
            stopSelf();
            return START_NOT_STICKY;
        }
        if ((commandThread == null || !commandThread.isAlive()) && tcpBridge == null)
            startCommandReceiver();
        if (heartbeatThread == null || !heartbeatThread.isAlive()) startHeartbeat();
        return START_STICKY;
    }

    @Override public void onDestroy() {
        commandRunning = false;
        heartbeatRunning = false;
        if (tcpBridge != null) tcpBridge.stop();
        if (receiver != null) unregisterReceiver(receiver);
        closePort();
        stopForeground(true);
        super.onDestroy();
    }

    @Override public IBinder onBind(Intent intent) { return null; }
}
