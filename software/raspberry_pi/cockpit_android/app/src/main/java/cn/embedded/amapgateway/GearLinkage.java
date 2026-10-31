package cn.embedded.amapgateway;

import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.content.ComponentName;
import android.os.Handler;
import android.os.HandlerThread;
import android.util.Log;

import org.json.JSONException;
import org.json.JSONObject;

import java.util.Locale;

/**
 * 档位联动状态机：把收到的档位报文翻译成倒车影像的"显示/退出"动作。
 *
 * 职责边界只包含「车辆档位 -> 影像显示状态」，不关心报文来自 UART、广播还是
 * 本地自检；通道差异由调用方负责。
 *
 * 规则：
 *   R 档（gear=R 或 camera=reverse）        -> 弹出倒车影像
 *   离开 R 档（gear=P/N/D 或 camera=none）  -> 退出倒车影像
 *   报文未携带档位信息                       -> 忽略，避免误触发
 */
final class GearLinkage {
    private static final String TAG = "AmapUartGateway";
    /** 车载倒车影像工程的固定包名；R 档不得回退到系统相机。 */
    static final String DEFAULT_REVERSE_PACKAGE = "cn.embedded.dualcamera";
    private static final String DEFAULT_REVERSE_ACTIVITY =
            "cn.embedded.dualcamera.MainActivity";

    private final Context context;
    private final Handler worker;
    private boolean reverseVisible;
    private String targetPackage = DEFAULT_REVERSE_PACKAGE;
    private Boolean pendingReverse;
    private long transitionSerial;

    GearLinkage(Context context) {
        this.context = context.getApplicationContext();
        HandlerThread thread = new HandlerThread("gear-linkage");
        thread.start();
        this.worker = new Handler(thread.getLooper());
    }

    /** 解析一行 UART 控制报文；无法识别时只记日志，不抛异常。 */
    void onControlLine(String line) {
        if (line == null) return;
        String text = line.trim();
        if (text.isEmpty()) return;

        String gear = null;
        String camera = null;
        if (text.startsWith("{")) {
            try {
                JSONObject object = new JSONObject(text);
                if (object.has("gear")) gear = object.optString("gear", "");
                if (object.has("camera")) camera = object.optString("camera", "");
            } catch (JSONException error) {
                Log.w(TAG, "忽略无法解析的控制报文：" + text);
                return;
            }
        } else {
            gear = text;  // 兼容直接下发裸档位字符（如 "R"）的简易报文
        }
        onGear(gear, camera);
    }

    /** 档位状态机入口，线程安全，可在任意线程调用。 */
    synchronized void onGear(String gear, String camera) {
        final Boolean wantReverse = resolveWantReverse(gear, camera);
        if (wantReverse == null) return;
        // UART/广播可能在同一时刻连续到达 R→D→R。旧实现把每条命令都排队，
        // 于是 D 已退出后，队列里的旧 R 又会把倒车影像拉起，用户看到闪屏。
        // 这里只执行最后一个稳定目标，并让每次新目标使旧回调失效。
        final long serial = ++transitionSerial;
        pendingReverse = wantReverse;
        worker.removeCallbacksAndMessages(null);
        worker.postDelayed(() -> {
            if (serial != transitionSerial || pendingReverse == null) return;
            if (pendingReverse) showReverse();
            else hideReverse();
        }, 120);
    }

    /** @return TRUE 显示 / FALSE 退出 / null 报文无档位信息，忽略 */
    private static Boolean resolveWantReverse(String gear, String camera) {
        String gearText = gear == null ? "" : gear.trim();
        String cameraText = camera == null ? "" : camera.trim().toLowerCase(Locale.ROOT);
        if (gearText.isEmpty() && cameraText.isEmpty()) return null;
        // R 档优先级最高：即使同时带导航转向画面也必须是倒车影像
        if ("R".equalsIgnoreCase(gearText) || "reverse".equals(cameraText)) return Boolean.TRUE;
        if ("none".equals(cameraText)) return Boolean.FALSE;
        if (!gearText.isEmpty()) return Boolean.FALSE;
        return null;
    }

    private void showReverse() {
        if (reverseVisible) return;
        Intent intent = resolveReverseIntent();
        if (intent == null) {
            Log.e(TAG, "系统内找不到可用倒车影像应用，无法显示画面");
            return;
        }
        intent.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK | Intent.FLAG_ACTIVITY_SINGLE_TOP);
        try {
            context.startActivity(intent);
            reverseVisible = true;
            Log.i(TAG, "R 档：已弹出倒车影像 -> " + intent.getComponent());
        } catch (RuntimeException error) {
            reverseVisible = false;
            Log.e(TAG, "R 档：倒车影像被系统拦截，请确认已授予悬浮窗/后台弹出界面权限", error);
        }
    }

    private void hideReverse() {
        if (!reverseVisible) return;
        reverseVisible = false;
        // 优先直接结束倒车影像进程。旧实现先 startActivity(exit)，会把倒车
        // Activity 再启动/置顶一次，D 挡时因此出现“退出→闪一下→又回倒车影像”。
        if (runAsRoot("am force-stop " + shellQuote(targetPackage))) {
            Log.i(TAG, "离开 R 档：已停止倒车影像应用");
            return;
        }
        // 无 root 时才回退到 Activity 自己 finish 的兼容路径。
        try {
            Intent exit = new Intent();
            exit.setComponent(new ComponentName(DEFAULT_REVERSE_PACKAGE, DEFAULT_REVERSE_ACTIVITY));
            exit.putExtra("exit_reverse", true);
            exit.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK | Intent.FLAG_ACTIVITY_CLEAR_TOP
                    | Intent.FLAG_ACTIVITY_SINGLE_TOP);
            context.startActivity(exit);
            Log.i(TAG, "离开 R 档：已请求倒车影像返回上一个界面");
            return;
        } catch (RuntimeException error) {
            Log.w(TAG, "倒车 Activity 退出请求失败，尝试停止应用", error);
        }
    }

    private Intent resolveReverseIntent() {
        PackageManager manager = context.getPackageManager();
        try {
            manager.getPackageInfo(DEFAULT_REVERSE_PACKAGE, 0);
        } catch (PackageManager.NameNotFoundException missing) {
            Log.e(TAG, "未安装自定义倒车影像 APK: " + DEFAULT_REVERSE_PACKAGE);
            return null;
        }

        Intent launch = new Intent();
        launch.setComponent(new ComponentName(DEFAULT_REVERSE_PACKAGE, DEFAULT_REVERSE_ACTIVITY));
        launch.putExtra("fullscreen_reverse", true);
        targetPackage = DEFAULT_REVERSE_PACKAGE;
        return launch;
    }

    private static boolean runAsRoot(String command) {
        try {
            Process process = new ProcessBuilder("su", "0", "sh", "-c", command).start();
            return process.waitFor() == 0;
        } catch (Exception error) {
            Log.w(TAG, "执行 root 命令失败：" + command, error);
            return false;
        }
    }

    static String shellQuote(String value) {
        return "'" + value.replace("'", "'\\''") + "'";
    }
}
