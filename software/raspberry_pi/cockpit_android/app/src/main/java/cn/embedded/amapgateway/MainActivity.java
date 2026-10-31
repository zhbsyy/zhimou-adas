package cn.embedded.amapgateway;

import android.app.Activity;
import android.content.Intent;
import android.graphics.Color;
import android.graphics.Typeface;
import android.graphics.drawable.GradientDrawable;
import android.net.Uri;
import android.os.Build;
import android.os.Bundle;
import android.provider.Settings;
import android.view.Gravity;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.Spinner;
import android.widget.TextView;

/** 网关控制台：只允许从预设硬件清单中选择串口。 */
public final class MainActivity extends Activity {
    static final String PREFS = "gateway_settings";

    private static final int BG = Color.rgb(9, 14, 27);
    private static final int PANEL = Color.rgb(19, 29, 49);
    private static final int PANEL_ALT = Color.rgb(24, 38, 63);
    private static final int TEXT = Color.rgb(235, 242, 255);
    private static final int MUTED = Color.rgb(145, 163, 190);
    private static final int CYAN = Color.rgb(44, 210, 230);
    private static final int GREEN = Color.rgb(69, 214, 139);
    private static final int AMBER = Color.rgb(255, 184, 77);
    private static final int RED = Color.rgb(239, 102, 115);

    private static final String[] UART_OPTIONS = {
            "tcp://10.0.2.2:8765（Android 模拟器桥接）",
            "/dev/ttyAMA0（树莓派40Pin 8/10）", "/dev/ttyAMA10", "/dev/ttyS0", "/dev/ttyS1",
            "/dev/ttyS3", "/dev/ttyS4", "/dev/ttyS7", "/dev/ttyUSB0"
    };
    private static final String[] BAUD_OPTIONS = {"115200", "9600", "57600", "38400", "230400"};
    private static final String[] CAMERA_OPTIONS = {
            "自定义倒车影像 · cn.embedded.dualcamera"
    };
    private static final String[] CAMERA_PACKAGES = {
            "cn.embedded.dualcamera"
    };

    private Spinner portPicker;
    private Spinner baudPicker;
    private Spinner cameraPicker;
    private TextView gatewayState;
    private TextView overlayState;
    private TextView routeSummary;
    private GearLinkage selfTest;
    private boolean selfTestReverseVisible;

    @Override protected void onCreate(Bundle state) {
        super.onCreate(state);
        getSharedPreferences(PREFS, MODE_PRIVATE).edit().putBoolean("enabled", true).apply();
        startGatewayService();
        getWindow().getDecorView().setSystemUiVisibility(5894 | 1024 | 512);

        ScrollView scroll = new ScrollView(this);
        scroll.setFillViewport(true);
        scroll.setBackgroundColor(BG);

        LinearLayout page = column();
        page.setPadding(dp(28), dp(24), dp(28), dp(24));
        scroll.addView(page, new ScrollView.LayoutParams(-1, -1));

        page.addView(header(), match());
        page.addView(summaryRow(), marginTop(match(), 18));
        page.addView(configSection(), marginTop(match(), 18));
        page.addView(operationSection(), marginTop(match(), 18));
        page.addView(footer(), marginTop(match(), 18));
        setContentView(scroll);
        refreshPermissionStatus();
    }

    private View header() {
        LinearLayout box = panel(PANEL, 24);
        box.addView(label("LINK CONTROL / 5AA5", 11, CYAN, true), match());
        box.addView(label("航迹桥接终端", 30, TEXT, true), marginTop(match(), 5));
        box.addView(label("导航广播 · 串口帧转发 · R 档画面联动", 15, MUTED, false),
                marginTop(match(), 8));
        return box;
    }

    private View summaryRow() {
        LinearLayout row = new LinearLayout(this);
        row.setOrientation(LinearLayout.HORIZONTAL);
        row.setWeightSum(3f);
        row.addView(metric("通道", isEmulator() ? "TCP → USB-TTL" : "UART / READY", GREEN), weight(1));
        row.addView(metric("协议", "5A A5 · 36 B", CYAN), marginWeight(1, 12));
        row.addView(metric("触发源", "AMAP 10001", AMBER), marginWeight(1, 12));
        return row;
    }

    private View metric(String caption, String value, int accent) {
        LinearLayout box = panel(PANEL_ALT, 18);
        View bar = new View(this);
        bar.setBackgroundColor(accent);
        box.addView(bar, new LinearLayout.LayoutParams(dp(46), dp(4)));
        box.addView(label(caption, 11, MUTED, true), marginTop(match(), 12));
        box.addView(label(value, 17, TEXT, true), marginTop(match(), 4));
        return box;
    }

    private View configSection() {
        LinearLayout card = panel(PANEL, 24);
        card.addView(label("设备路由", 19, TEXT, true), match());
        card.addView(label("串口节点固定为设备清单，禁止自由输入。", 13, MUTED, false),
                marginTop(match(), 5));

        LinearLayout fields = new LinearLayout(this);
        fields.setOrientation(LinearLayout.HORIZONTAL);
        fields.setWeightSum(3f);
        portPicker = spinner(UART_OPTIONS);
        baudPicker = spinner(BAUD_OPTIONS);
        cameraPicker = spinner(CAMERA_OPTIONS);
        fields.addView(field("串口路由", portPicker), weight(1));
        fields.addView(field("速率", baudPicker), marginWeight(1, 12));
        fields.addView(field("倒车画面目标", cameraPicker), marginWeight(1, 12));
        card.addView(fields, marginTop(match(), 16));
        restoreSelections();

        routeSummary = label("", 13, CYAN, true);
        card.addView(routeSummary, marginTop(match(), 14));
        updateRouteSummary();
        portPicker.setOnItemSelectedListener(new SimpleItemListener(this::updateRouteSummary));
        baudPicker.setOnItemSelectedListener(new SimpleItemListener(this::updateRouteSummary));
        cameraPicker.setOnItemSelectedListener(new SimpleItemListener(this::updateRouteSummary));
        return card;
    }

    private View operationSection() {
        LinearLayout card = panel(PANEL, 24);
        card.addView(label("操作台", 19, TEXT, true), match());
        card.addView(label("运行状态与联动动作", 13, MUTED, false), marginTop(match(), 5));

        LinearLayout statusRow = new LinearLayout(this);
        statusRow.setOrientation(LinearLayout.HORIZONTAL);
        statusRow.setGravity(Gravity.CENTER_VERTICAL);
        gatewayState = label("●  链路待命", 16, GREEN, true);
        statusRow.addView(gatewayState, weight(1));
        overlayState = label("", 13, MUTED, false);
        statusRow.addView(overlayState, wrap());
        card.addView(statusRow, marginTop(match(), 16));

        LinearLayout actions = new LinearLayout(this);
        actions.setOrientation(LinearLayout.HORIZONTAL);
        actions.setWeightSum(3f);
        actions.addView(action("启用链路", CYAN, v -> startGateway()), weight(1));
        actions.addView(action("暂停链路", RED, v -> stopGateway()), marginWeight(1, 12));
        actions.addView(action("权限配置", AMBER, v -> requestOverlayPermission()), marginWeight(1, 12));
        card.addView(actions, marginTop(match(), 14));

        LinearLayout tools = new LinearLayout(this);
        tools.setOrientation(LinearLayout.HORIZONTAL);
        tools.setWeightSum(4f);
        tools.addView(action("返回导航", CYAN, v -> finish()), weight(1));
        tools.addView(action("返回桌面", GREEN, v -> returnToHome()), marginWeight(1, 12));
        tools.addView(action("R 档演练", Color.rgb(171, 121, 255), v -> toggleSelfTestReverse()),
                marginWeight(1, 12));
        tools.addView(action("退出应用", RED, v -> exitApp()), marginWeight(1, 12));
        card.addView(tools, marginTop(match(), 12));
        return card;
    }

    private View footer() {
        TextView note = label(isEmulator()
                        ? "模拟器通过 10.0.2.2:8765 连接电脑 A 的串口桥；COM10 和 COM11 之间走真实 3.3V TTL 接线。"
                        : "实体座舱请选择对应的 Linux UART 设备节点。",
                12, MUTED, false);
        note.setGravity(Gravity.CENTER);
        return note;
    }

    private Spinner spinner(String[] values) {
        Spinner spinner = new Spinner(this);
        ArrayAdapter<String> adapter = new ArrayAdapter<String>(
                this, android.R.layout.simple_spinner_item, values) {
            @Override public View getView(int position, View convertView, ViewGroup parent) {
                TextView view = (TextView) super.getView(position, convertView, parent);
                view.setTextColor(TEXT);
                view.setTextSize(15);
                view.setPadding(dp(14), dp(11), dp(14), dp(11));
                return view;
            }
        };
        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        spinner.setAdapter(adapter);
        spinner.setBackground(roundStroke(Color.rgb(42, 62, 91), Color.rgb(31, 47, 74), 14, 1));
        return spinner;
    }

    private View field(String title, Spinner spinner) {
        LinearLayout box = column();
        box.addView(label(title, 11, MUTED, true), match());
        box.addView(spinner, marginTop(match(), 7));
        return box;
    }

    private Button action(String text, int color, View.OnClickListener listener) {
        Button button = new Button(this);
        button.setText(text);
        button.setTextColor(BG);
        button.setTextSize(14);
        button.setTypeface(Typeface.DEFAULT, Typeface.BOLD);
        button.setAllCaps(false);
        button.setGravity(Gravity.CENTER);
        button.setPadding(dp(8), dp(3), dp(8), dp(3));
        button.setBackground(roundStroke(color, color, 16, 0));
        button.setOnClickListener(listener);
        return button;
    }

    private LinearLayout panel(int color, int radius) {
        LinearLayout box = column();
        box.setPadding(dp(20), dp(18), dp(20), dp(18));
        box.setBackground(roundStroke(color, color, radius, 0));
        return box;
    }

    private LinearLayout column() {
        LinearLayout box = new LinearLayout(this);
        box.setOrientation(LinearLayout.VERTICAL);
        return box;
    }

    private TextView label(String text, float size, int color, boolean bold) {
        TextView view = new TextView(this);
        view.setText(text);
        view.setTextColor(color);
        view.setTextSize(size);
        view.setTypeface(Typeface.DEFAULT, bold ? Typeface.BOLD : Typeface.NORMAL);
        return view;
    }

    private GradientDrawable roundStroke(int fill, int stroke, int radius, int strokeWidth) {
        GradientDrawable drawable = new GradientDrawable();
        drawable.setColor(fill);
        drawable.setCornerRadius(dp(radius));
        if (strokeWidth > 0) drawable.setStroke(dp(strokeWidth), stroke);
        return drawable;
    }

    private void restoreSelections() {
        String savedPort = getSharedPreferences(PREFS, MODE_PRIVATE).getString("port", "");
        if (isEmulator() && !savedPort.startsWith("tcp://")) savedPort = "tcp://10.0.2.2:8765";
        if (savedPort.isEmpty()) savedPort = "/dev/ttyAMA0";
        if ("/dev/ttyS3".equals(savedPort) || "/dev/ttyAMA10".equals(savedPort)) savedPort = "/dev/ttyAMA0";
        String savedBaud = String.valueOf(getSharedPreferences(PREFS, MODE_PRIVATE)
                .getInt("baud", 115200));
        String savedCamera = getSharedPreferences(PREFS, MODE_PRIVATE)
                .getString("reverse_package", GearLinkage.DEFAULT_REVERSE_PACKAGE);
        portPicker.setSelection(indexOf(UART_OPTIONS, savedPort));
        baudPicker.setSelection(indexOf(BAUD_OPTIONS, savedBaud));
        cameraPicker.setSelection(0);
    }

    private void updateRouteSummary() {
        if (routeSummary == null || portPicker == null) return;
        String port = portValue();
        String baud = baudPicker.getSelectedItem().toString();
        String camera = CAMERA_PACKAGES[cameraPicker.getSelectedItemPosition()];
        routeSummary.setText("当前路由  ·  " + (port.isEmpty() ? "AUTO" : port)
                + "  /  " + baud + " Bd  /  "
                + (camera.isEmpty() ? "系统视频 Intent" : camera));
    }

    private String portValue() {
        String selected = portPicker.getSelectedItem().toString();
        if (selected.startsWith("tcp://10.0.2.2:8765")) return "tcp://10.0.2.2:8765";
        return selected.startsWith("/dev/ttyAMA0") ? "/dev/ttyAMA0" : selected;
    }

    private static boolean isEmulator() {
        return Build.FINGERPRINT.startsWith("generic") || Build.FINGERPRINT.contains("emulator")
                || Build.MODEL.contains("Emulator") || Build.PRODUCT.contains("sdk_gphone");
    }

    private void startGateway() {
        String port = portValue();
        int baud = Integer.parseInt(baudPicker.getSelectedItem().toString());
        String camera = CAMERA_PACKAGES[cameraPicker.getSelectedItemPosition()];
        getSharedPreferences(PREFS, MODE_PRIVATE).edit()
                .putString("port", port)
                .putInt("baud", baud)
                .putString("reverse_package", camera)
                .putBoolean("enabled", true).apply();
        startGatewayService();
        gatewayState.setText("●  链路运行中");
        gatewayState.setTextColor(GREEN);
        updateRouteSummary();
    }

    private void stopGateway() {
        startService(new Intent(this, GatewayService.class).setAction(GatewayService.ACTION_STOP));
        getSharedPreferences(PREFS, MODE_PRIVATE).edit().putBoolean("enabled", false).apply();
        gatewayState.setText("●  链路已暂停");
        gatewayState.setTextColor(RED);
    }

    private void startGatewayService() {
        Intent service = new Intent(this, GatewayService.class);
        if (Build.VERSION.SDK_INT >= 26) startForegroundService(service);
        else startService(service);
    }

    private void launchAmap() {
        Intent launch = getPackageManager().getLaunchIntentForPackage("com.autonavi.amapauto");
        if (launch != null) startActivity(launch);
        else gatewayState.setText("●  未找到导航应用");
    }

    @Override protected void onResume() {
        super.onResume();
        refreshPermissionStatus();
    }

    private void refreshPermissionStatus() {
        if (overlayState == null) return;
        boolean allowed = Settings.canDrawOverlays(this);
        overlayState.setText(allowed ? "后台弹窗：已允许" : "后台弹窗：待授权");
        overlayState.setTextColor(allowed ? GREEN : AMBER);
    }

    private void requestOverlayPermission() {
        Intent intent = new Intent(Settings.ACTION_MANAGE_OVERLAY_PERMISSION,
                Uri.parse("package:" + getPackageName()));
        try {
            startActivity(intent);
            gatewayState.setText("●  请完成后台弹窗授权");
        } catch (Exception error) {
            gatewayState.setText("●  系统未提供权限页");
        }
    }

    private void toggleSelfTestReverse() {
        if (selfTest == null) selfTest = new GearLinkage(this);
        selfTestReverseVisible = !selfTestReverseVisible;
        selfTest.onGear(selfTestReverseVisible ? "R" : "D",
                selfTestReverseVisible ? "reverse" : "none");
        gatewayState.setText(selfTestReverseVisible ? "●  R 档演练：画面请求已发出"
                : "●  R 档演练：已退出");
    }

    private void closePanel() {
        if (isTaskRoot()) finish();
        else onBackPressed();
    }

    private void returnToHome() {
        Intent home = new Intent(Intent.ACTION_MAIN);
        home.addCategory(Intent.CATEGORY_HOME);
        home.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
        startActivity(home);
    }

    private void exitApp() {
        finishAffinity();
        System.exit(0);
    }

    private int indexOf(String[] values, String target) {
        for (int i = 0; i < values.length; i++) if (values[i].equals(target) || values[i].startsWith(target + "（")) return i;
        return 0;
    }

    private int dp(int value) {
        return Math.round(value * getResources().getDisplayMetrics().density);
    }

    private LinearLayout.LayoutParams match() {
        return new LinearLayout.LayoutParams(-1, -2);
    }

    private LinearLayout.LayoutParams wrap() {
        return new LinearLayout.LayoutParams(-2, -2);
    }

    private LinearLayout.LayoutParams weight(float value) {
        return new LinearLayout.LayoutParams(0, -2, value);
    }

    private LinearLayout.LayoutParams marginTop(LinearLayout.LayoutParams params, int value) {
        params.topMargin = dp(value);
        return params;
    }

    private LinearLayout.LayoutParams marginWeight(float weight, int margin) {
        LinearLayout.LayoutParams params = weight(weight);
        params.leftMargin = dp(margin);
        return params;
    }

    private static final class SimpleItemListener implements android.widget.AdapterView.OnItemSelectedListener {
        private final Runnable callback;

        SimpleItemListener(Runnable callback) { this.callback = callback; }

        @Override public void onItemSelected(android.widget.AdapterView<?> parent, View view,
                                             int position, long id) { callback.run(); }

        @Override public void onNothingSelected(android.widget.AdapterView<?> parent) { }
    }
}
