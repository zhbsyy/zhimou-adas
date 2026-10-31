package cn.embedded.amapgateway;

import android.app.Activity;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.graphics.Color;
import android.graphics.Typeface;
import android.graphics.drawable.GradientDrawable;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.view.Gravity;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.HorizontalScrollView;
import android.widget.LinearLayout;
import android.widget.ProgressBar;
import android.widget.TextView;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * 原生小镇自主寻路座舱导航系统。
 * 彻底移除固定路线，支持从当前自车位置到地图任意点击点/POI的实时最短路径规划与上位机自主闭环循迹。
 */
public final class CockpitNavActivity extends Activity {

    private static final int COLOR_CARD_BG = Color.argb(225, 17, 24, 39);
    private static final int COLOR_CYAN = Color.rgb(44, 210, 230);
    private static final int COLOR_GREEN = Color.rgb(0, 230, 118);
    private static final int COLOR_AMBER = Color.rgb(255, 184, 77);
    private static final int COLOR_RED = Color.rgb(239, 68, 68);

    private CockpitMapView mapView;
    private TownMapData mapData;
    private final Handler handler = new Handler();

    // 默认起点：都会立交枢纽
    private float currentVehX = -4.0f;
    private float currentVehY = 24.7f;
    private float currentVehYaw = 0.2f;

    // 当前导航目标
    private float targetDestX = 98.8f;
    private float targetDestY = 82.9f;
    private String targetDestName = "东岸CBD金融中心";

    private int currentSpeedLimitKmh = 40;
    private int actualSpeedKmh = 0;
    private int targetSpeedKmh = 40;
    private String currentGear = "P";
    private boolean isNavigating = false;
    private int segRemainDistM = 35;
    private int routeRemainDistM = 154;
    private String maneuverText = "沿当前车道平稳直行";
    private int navIcon = 1;
    private boolean aebActive = false;
    private long lastRerouteTime = 0L;

    // UI 控件
    private TextView hudMainText;
    private TextView hudSubText;
    private TextView hudIconView;
    private TextView hudSpeedLimitView;
    private ProgressBar hudProgressBar;

    private TextView speedNumberView;
    private TextView linkStatusView;
    private View aebAlertBanner;
    private TextView aebAlertText;

    private final Button[] gearButtons = new Button[4];
    private final String[] gearNames = {"P", "R", "N", "D"};
    private Button btnNavToggle;
    private TextView destStatusText;

    private GearLinkage gearLinkage;

    private final BroadcastReceiver oibStateReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            if (GatewayService.LINK_STATE.equals(intent.getAction())) {
                boolean connected = intent.getBooleanExtra("connected", false);
                if (linkStatusView != null) {
                    linkStatusView.setText(connected ? "● 串口桥已连接" : "● 串口桥未连接");
                    linkStatusView.setTextColor(connected ? COLOR_GREEN : COLOR_AMBER);
                }
                return;
            }
            if ("cn.embedded.amapgateway.ROUTE_SYNC".equals(intent.getAction())) {
                float[] flat = intent.getFloatArrayExtra("flat_waypoints");
                if (flat != null && flat.length >= 4) {
                    List<float[]> wps = new ArrayList<>();
                    for (int i = 0; i < flat.length; i += 2) {
                        wps.add(new float[]{flat[i], flat[i + 1], 0.0f});
                    }
                    mapView.setNavigationPath(wps);
                    float totalM = intent.getFloatExtra("total_m", 0.0f);
                    if (totalM > 0) {
                        routeRemainDistM = Math.round(totalM);
                        segRemainDistM = Math.min(routeRemainDistM, 35);
                        if (destStatusText != null) {
                            destStatusText.setText(String.format(Locale.US, "目的地: %s (全程约 %dm)", targetDestName, routeRemainDistM));
                        }
                    }
                }
                return;
            }

            String incomingGear = intent.getStringExtra("gear");
            if (incomingGear != null && !incomingGear.isEmpty()) {
                currentGear = incomingGear.toUpperCase(Locale.ROOT);
            }

            int incomingActual = intent.getIntExtra("actual_speed_kmh", -1);
            if (incomingActual >= 0) actualSpeedKmh = incomingActual;

            int incomingTarget = intent.getIntExtra("target_speed_kmh", -1);
            if (incomingTarget >= 0) targetSpeedKmh = incomingTarget;

            int incomingSeg = intent.getIntExtra("segment_remaining_distance_m", -1);
            if (incomingSeg >= 0) segRemainDistM = incomingSeg;

            int incomingRoute = intent.getIntExtra("route_remaining_distance_m", -1);
            if (incomingRoute >= 0) routeRemainDistM = incomingRoute;

            if (intent.hasExtra("simulation_running")) {
                isNavigating = intent.getBooleanExtra("simulation_running", isNavigating);
            }

            // 读取上位机回传的自车真实位姿
            if (intent.hasExtra("pos_x") && intent.hasExtra("pos_y")) {
                currentVehX = intent.getFloatExtra("pos_x", currentVehX);
                currentVehY = intent.getFloatExtra("pos_y", currentVehY);
                currentVehYaw = intent.getFloatExtra("heading_deg", currentVehYaw);
                mapView.updateVehiclePose(currentVehX, currentVehY, currentVehYaw);
                checkOffRouteAndReroute();
            }

            // 严格仅在真实 AEB 触发时报警
            aebActive = intent.getBooleanExtra("aeb_active", false);
            float pedDist = intent.getFloatExtra("pedestrian_distance_m", 0.0f);
            mapView.updatePedestrianAlert(false, pedDist, 0.0f, aebActive);

            updateHUD();
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        // 保持座舱屏幕常亮与全屏沉浸
        getWindow().addFlags(android.view.WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        getWindow().getDecorView().setSystemUiVisibility(
                View.SYSTEM_UI_FLAG_FULLSCREEN | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                        | View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                        | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION);

        gearLinkage = new GearLinkage(this);
        startGatewayService();

        FrameLayout root = new FrameLayout(this);
        root.setBackgroundColor(Color.BLACK);

        // 1. 底层地图画布
        mapView = new CockpitMapView(this);
        mapData = TownMapData.loadFromAssets(this);
        mapView.setMapData(mapData);
        root.addView(mapView, new FrameLayout.LayoutParams(-1, -1));

        // 触控选点导航监听：点击地图任何位置立即计算自主路径
        mapView.setOnLocationSelectedListener((worldX, worldY, description) -> {
            navigateToDestination(worldX, worldY, description);
        });

        // 2. 覆盖层 UI
        createOverlayUI(root);

        setContentView(root);

        // 注册 OIB 状态监听与导航轨迹同步
        IntentFilter filter = new IntentFilter("cn.embedded.amapgateway.OIB_STATE");
        filter.addAction("cn.embedded.amapgateway.ROUTE_SYNC");
        filter.addAction(GatewayService.LINK_STATE);
        if (Build.VERSION.SDK_INT >= 33) registerReceiver(oibStateReceiver, filter, Context.RECEIVER_EXPORTED);
        else registerReceiver(oibStateReceiver, filter);

        // 初始化默认起点为都会立交，默认规划至东岸CBD
        resetToDefaultSpawn();
        startLocalTick();
    }

    private void startGatewayService() {
        Intent service = new Intent(this, GatewayService.class);
        if (Build.VERSION.SDK_INT >= 26) startForegroundService(service);
        else startService(service);
    }

    private void startLocalTick() {
        handler.postDelayed(new Runnable() {
            @Override
            public void run() {
                if (isNavigating && "D".equals(currentGear)) {
                    broadcastAmapNavigation();
                }
                handler.postDelayed(this, 1000);
            }
        }, 1000);
    }

    @Override
    protected void onDestroy() {
        handler.removeCallbacksAndMessages(null);
        try {
            unregisterReceiver(oibStateReceiver);
        } catch (Exception ignored) {}
        super.onDestroy();
    }

    private void createOverlayUI(FrameLayout root) {
        // 顶部 HUD 导航引导卡片
        View topHUD = buildTopGuidanceCard();
        FrameLayout.LayoutParams topLp = new FrameLayout.LayoutParams(dp(540), -2);
        topLp.gravity = Gravity.TOP | Gravity.CENTER_HORIZONTAL;
        topLp.topMargin = dp(18);
        root.addView(topHUD, topLp);

        // 顶部 AEB 紧急制动警报条
        aebAlertBanner = buildAebAlertBanner();
        FrameLayout.LayoutParams alertLp = new FrameLayout.LayoutParams(dp(620), dp(44));
        alertLp.gravity = Gravity.TOP | Gravity.CENTER_HORIZONTAL;
        alertLp.topMargin = dp(96);
        root.addView(aebAlertBanner, alertLp);

        // 左侧自车仪表 HUD
        View leftHUD = buildLeftVehicleHUD();
        FrameLayout.LayoutParams leftLp = new FrameLayout.LayoutParams(dp(220), -2);
        leftLp.gravity = Gravity.TOP | Gravity.START;
        leftLp.topMargin = dp(18);
        leftLp.leftMargin = dp(20);
        root.addView(leftHUD, leftLp);

        // 右侧地图控制悬浮按钮
        View rightControls = buildRightControls();
        FrameLayout.LayoutParams rightLp = new FrameLayout.LayoutParams(-2, -2);
        rightLp.gravity = Gravity.TOP | Gravity.END;
        rightLp.topMargin = dp(18);
        rightLp.rightMargin = dp(20);
        root.addView(rightControls, rightLp);

        // 底部自主寻路控制抽屉
        View bottomBar = buildBottomControlBar();
        FrameLayout.LayoutParams bottomLp = new FrameLayout.LayoutParams(-1, -2);
        bottomLp.gravity = Gravity.BOTTOM;
        bottomLp.bottomMargin = dp(14);
        bottomLp.leftMargin = dp(20);
        bottomLp.rightMargin = dp(20);
        root.addView(bottomBar, bottomLp);
    }

    // ------------------------------------------------------------- 顶部导航 HUD
    private View buildTopGuidanceCard() {
        LinearLayout card = new LinearLayout(this);
        card.setOrientation(LinearLayout.HORIZONTAL);
        card.setGravity(Gravity.CENTER_VERTICAL);
        card.setPadding(dp(18), dp(12), dp(18), dp(12));
        card.setBackground(roundRect(COLOR_CARD_BG, 18, Color.argb(40, 255, 255, 255), 1));

        hudIconView = new TextView(this);
        hudIconView.setText("↑");
        hudIconView.setTextSize(34);
        hudIconView.setTypeface(Typeface.DEFAULT, Typeface.BOLD);
        hudIconView.setTextColor(COLOR_GREEN);
        hudIconView.setGravity(Gravity.CENTER);
        card.addView(hudIconView, new LinearLayout.LayoutParams(dp(50), dp(50)));

        LinearLayout mid = new LinearLayout(this);
        mid.setOrientation(LinearLayout.VERTICAL);
        mid.setPadding(dp(12), 0, dp(12), 0);

        hudMainText = new TextView(this);
        hudMainText.setText("前方 35 米 直行");
        hudMainText.setTextSize(18);
        hudMainText.setTypeface(Typeface.DEFAULT, Typeface.BOLD);
        hudMainText.setTextColor(Color.WHITE);
        mid.addView(hudMainText);

        hudSubText = new TextView(this);
        hudSubText.setText("全程剩余 154 米 · 限速 40 km/h");
        hudSubText.setTextSize(12);
        hudSubText.setTextColor(Color.rgb(156, 163, 175));
        mid.addView(hudSubText, margin(0, dp(2), 0, 0));

        hudProgressBar = new ProgressBar(this, null, android.R.attr.progressBarStyleHorizontal);
        hudProgressBar.setMax(100);
        hudProgressBar.setProgress(10);
        mid.addView(hudProgressBar, margin(0, dp(4), 0, 0));

        card.addView(mid, new LinearLayout.LayoutParams(0, -2, 1.0f));

        hudSpeedLimitView = new TextView(this);
        hudSpeedLimitView.setText("40");
        hudSpeedLimitView.setTextSize(16);
        hudSpeedLimitView.setTypeface(Typeface.DEFAULT, Typeface.BOLD);
        hudSpeedLimitView.setTextColor(Color.BLACK);
        hudSpeedLimitView.setGravity(Gravity.CENTER);
        hudSpeedLimitView.setBackground(roundStroke(Color.WHITE, Color.RED, 22, 3));
        card.addView(hudSpeedLimitView, new LinearLayout.LayoutParams(dp(44), dp(44)));

        return card;
    }

    private View buildAebAlertBanner() {
        LinearLayout banner = new LinearLayout(this);
        banner.setOrientation(LinearLayout.HORIZONTAL);
        banner.setGravity(Gravity.CENTER);
        banner.setPadding(dp(16), dp(8), dp(16), dp(8));
        banner.setBackground(roundRect(Color.argb(230, 220, 38, 38), 12, Color.WHITE, 1));
        banner.setVisibility(View.GONE);

        aebAlertText = new TextView(this);
        aebAlertText.setText("[AEB] 视觉紧急制动介入中！已制动减速避让行人");
        aebAlertText.setTextSize(14);
        aebAlertText.setTypeface(Typeface.DEFAULT, Typeface.BOLD);
        aebAlertText.setTextColor(Color.WHITE);
        banner.addView(aebAlertText);

        return banner;
    }

    private View buildLeftVehicleHUD() {
        LinearLayout panel = new LinearLayout(this);
        panel.setOrientation(LinearLayout.VERTICAL);
        panel.setPadding(dp(16), dp(14), dp(16), dp(14));
        panel.setBackground(roundRect(COLOR_CARD_BG, 18, Color.argb(30, 255, 255, 255), 1));

        LinearLayout spdBox = new LinearLayout(this);
        spdBox.setOrientation(LinearLayout.HORIZONTAL);
        spdBox.setGravity(Gravity.BOTTOM);

        speedNumberView = new TextView(this);
        speedNumberView.setText("0");
        speedNumberView.setTextSize(44);
        speedNumberView.setTypeface(Typeface.DEFAULT, Typeface.BOLD);
        speedNumberView.setTextColor(COLOR_CYAN);
        spdBox.addView(speedNumberView);

        TextView spdUnit = new TextView(this);
        spdUnit.setText(" km/h");
        spdUnit.setTextSize(14);
        spdUnit.setTextColor(Color.rgb(156, 163, 175));
        spdUnit.setPadding(0, 0, 0, dp(8));
        spdBox.addView(spdUnit);

        panel.addView(spdBox);

        LinearLayout gearBox = new LinearLayout(this);
        gearBox.setOrientation(LinearLayout.HORIZONTAL);
        gearBox.setGravity(Gravity.CENTER);

        for (int i = 0; i < 4; i++) {
            final String g = gearNames[i];
            Button btn = new Button(this);
            btn.setText(g);
            btn.setTextSize(13);
            btn.setTypeface(Typeface.DEFAULT, Typeface.BOLD);
            btn.setPadding(0, 0, 0, 0);
            btn.setOnClickListener(v -> setGear(g));
            gearButtons[i] = btn;
            LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(dp(40), dp(34));
            if (i > 0) lp.leftMargin = dp(6);
            gearBox.addView(btn, lp);
        }
        panel.addView(gearBox, margin(0, dp(10), 0, 0));

        linkStatusView = new TextView(this);
        boolean linked = getSharedPreferences(MainActivity.PREFS, MODE_PRIVATE)
                .getBoolean("bridge_connected", false);
        linkStatusView.setText(linked ? "● 串口桥已连接" : "● 串口桥连接中");
        linkStatusView.setTextSize(11);
        linkStatusView.setTextColor(linked ? COLOR_GREEN : COLOR_AMBER);
        panel.addView(linkStatusView, margin(0, dp(10), 0, 0));

        return panel;
    }

    private View buildRightControls() {
        LinearLayout col = new LinearLayout(this);
        col.setOrientation(LinearLayout.VERTICAL);

        Button btnFollow = createRoundIconButton("视角", v -> {
            boolean cur = mapView.isFollowVehicle();
            mapView.setFollowVehicle(!cur);
        });
        col.addView(btnFollow);

        Button btnZoomIn = createRoundIconButton("+", v -> mapView.zoomIn());
        col.addView(btnZoomIn, margin(0, dp(10), 0, 0));

        Button btnZoomOut = createRoundIconButton("-", v -> mapView.zoomOut());
        col.addView(btnZoomOut, margin(0, dp(10), 0, 0));

        Button btnSettings = createRoundIconButton("设置", v -> {
            startActivity(new Intent(this, MainActivity.class));
        });
        col.addView(btnSettings, margin(0, dp(10), 0, 0));

        Button btnHome = createRoundIconButton("桌面", v -> returnToHome());
        col.addView(btnHome, margin(0, dp(10), 0, 0));

        Button btnExit = createRoundIconButton("关闭", v -> exitApp());
        btnExit.setTextColor(Color.rgb(239, 68, 68));
        col.addView(btnExit, margin(0, dp(10), 0, 0));

        return col;
    }

    private Button createRoundIconButton(String icon, View.OnClickListener l) {
        Button b = new Button(this);
        b.setText(icon);
        b.setTextSize(12);
        b.setTextColor(Color.WHITE);
        b.setPadding(0, 0, 0, 0);
        b.setBackground(roundRect(COLOR_CARD_BG, 22, Color.argb(60, 255, 255, 255), 1));
        b.setOnClickListener(l);
        return b;
    }

    // ------------------------------------------------------------- 底部自主导航控制栏
    private View buildBottomControlBar() {
        LinearLayout container = new LinearLayout(this);
        container.setOrientation(LinearLayout.VERTICAL);
        container.setPadding(dp(16), dp(10), dp(16), dp(10));
        container.setBackground(roundRect(COLOR_CARD_BG, 18, Color.argb(40, 255, 255, 255), 1));

        // 顶部信息与主控制按钮行
        LinearLayout topRow = new LinearLayout(this);
        topRow.setOrientation(LinearLayout.HORIZONTAL);
        topRow.setGravity(Gravity.CENTER_VERTICAL);

        destStatusText = new TextView(this);
        destStatusText.setText("目的地: 东岸CBD金融中心 (全程约 154m)");
        destStatusText.setTextSize(14);
        destStatusText.setTypeface(Typeface.DEFAULT, Typeface.BOLD);
        destStatusText.setTextColor(COLOR_CYAN);
        topRow.addView(destStatusText, new LinearLayout.LayoutParams(0, -2, 1.0f));

        btnNavToggle = new Button(this);
        btnNavToggle.setText("开始导航");
        btnNavToggle.setTextSize(14);
        btnNavToggle.setTypeface(Typeface.DEFAULT, Typeface.BOLD);
        btnNavToggle.setTextColor(Color.BLACK);
        btnNavToggle.setBackground(roundRect(COLOR_GREEN, 12, 0, 0));
        btnNavToggle.setPadding(dp(22), dp(6), dp(22), dp(6));
        btnNavToggle.setOnClickListener(v -> toggleNavigation());
        topRow.addView(btnNavToggle, new LinearLayout.LayoutParams(-2, dp(38)));

        Button btnReset = new Button(this);
        btnReset.setText("重置起点");
        btnReset.setTextSize(13);
        btnReset.setTextColor(Color.WHITE);
        btnReset.setBackground(roundRect(Color.rgb(55, 65, 81), 12, 0, 0));
        btnReset.setPadding(dp(14), dp(6), dp(14), dp(6));
        btnReset.setOnClickListener(v -> resetToDefaultSpawn());
        LinearLayout.LayoutParams rLp = new LinearLayout.LayoutParams(-2, dp(38));
        rLp.leftMargin = dp(10);
        topRow.addView(btnReset, rLp);
        container.addView(topRow);

        // 快捷目的地选择胶囊横向滑动条
        HorizontalScrollView scroll = new HorizontalScrollView(this);
        scroll.setHorizontalScrollBarEnabled(false);
        LinearLayout poiRow = new LinearLayout(this);
        poiRow.setOrientation(LinearLayout.HORIZONTAL);
        poiRow.setPadding(0, dp(8), 0, 0);

        if (mapData != null) {
            for (TownMapData.POI poi : mapData.pois) {
                if ("spawn".equals(poi.type)) continue;
                Button pb = new Button(this);
                pb.setText(poi.name);
                pb.setTextSize(12);
                pb.setTextColor(Color.rgb(226, 232, 240));
                pb.setBackground(roundRect(Color.rgb(31, 41, 55), 10, Color.argb(60, 255, 255, 255), 1));
                pb.setPadding(dp(12), dp(4), dp(12), dp(4));
                pb.setOnClickListener(v -> navigateToDestination(poi.x, poi.y, poi.name));
                LinearLayout.LayoutParams pLp = new LinearLayout.LayoutParams(-2, dp(34));
                pLp.rightMargin = dp(8);
                poiRow.addView(pb, pLp);
            }
        }
        scroll.addView(poiRow);
        container.addView(scroll);

        return container;
    }

    // ------------------------------------------------------------- 核心寻路逻辑
    /** 自主寻路：从自车当前位置规划路线到指定目的地 */
    private void navigateToDestination(float destX, float destY, String destName) {
        targetDestX = destX;
        targetDestY = destY;
        targetDestName = destName;

        if (mapData == null) return;
        TownMapData.RouteResult res = mapData.planRoute(currentVehX, currentVehY, destX, destY);

        if (res != null) {
            mapView.setNavigationPath(res.waypoints);
            mapView.setDestination(destX, destY, destName);
            routeRemainDistM = Math.round(res.totalDistanceM);
            segRemainDistM = Math.min(routeRemainDistM, 35);
            navIcon = res.initialTurnIcon;
            maneuverText = "前往 " + destName;

            if (destStatusText != null) {
                destStatusText.setText(String.format(Locale.US, "目的地: %s (全程约 %dm)", destName, routeRemainDistM));
            }
        }

        // 仅规划路线并通知上位机预载目标，保持待命（等待用户主动点击“开始导航”）
        isNavigating = false;
        sendSimulatorState();
        broadcastAmapNavigation();
        updateHUD();
    }

    /** 智能偏航检测与自车实时重寻路 (以自车实际位姿为准更新路线) */
    private void checkOffRouteAndReroute() {
        if (!isNavigating || mapData == null || mapView == null) return;
        long now = System.currentTimeMillis();
        if (now - lastRerouteTime < 3000L) return; // 节流控制：每次重算后至少间隔 3 秒

        List<float[]> path = mapView.getNavigationPath();
        if (path == null || path.size() < 2) return;

        // 计算自车当前位置到整条规划路径的最小横向距离
        float minD = Float.MAX_VALUE;
        for (float[] pt : path) {
            float d = (float) Math.hypot(pt[0] - currentVehX, pt[1] - currentVehY);
            if (d < minD) minD = d;
        }

        // 距终点小于 8 米时不触发重新寻路
        float dDest = (float) Math.hypot(targetDestX - currentVehX, targetDestY - currentVehY);
        if (dDest < 8.0f) return;

        // 若自车偏离规划路线超过 5.5 米（走错车道、提前或延迟转弯变道），以自车当前实际位置为起点即时重新规划
        if (minD > 5.5f) {
            TownMapData.RouteResult res = mapData.planRoute(currentVehX, currentVehY, targetDestX, targetDestY);
            if (res != null && res.waypoints.size() >= 2) {
                lastRerouteTime = now;
                mapView.setNavigationPath(res.waypoints);
                routeRemainDistM = Math.round(res.totalDistanceM);
                segRemainDistM = Math.min(routeRemainDistM, 35);
                navIcon = res.initialTurnIcon;
                maneuverText = "实时重算: 前往 " + targetDestName;
                if (destStatusText != null) {
                    destStatusText.setText(String.format(Locale.US, "目的地: %s (动态重算约 %dm)", targetDestName, routeRemainDistM));
                }
                sendSimulatorState();
                broadcastAmapNavigation();
                updateHUD();
            }
        }
    }

    /** 重置自车到默认起点（都会立交枢纽） */
    private void resetToDefaultSpawn() {
        if (mapData != null) {
            currentVehX = mapData.defaultSpawnX;
            currentVehY = mapData.defaultSpawnY;
            currentVehYaw = mapData.defaultSpawnYaw;
        }
        isNavigating = false;
        actualSpeedKmh = 0;
        setGear("P");
        mapView.updateVehiclePose(currentVehX, currentVehY, currentVehYaw);
        mapView.setFollowVehicle(true);

        // 默认规划至东岸CBD
        if (mapData != null) {
            TownMapData.RouteResult res = mapData.planRoute(currentVehX, currentVehY, 98.8f, 82.9f);
            if (res != null) {
                mapView.setNavigationPath(res.waypoints);
                mapView.setDestination(98.8f, 82.9f, "东岸CBD金融中心");
                routeRemainDistM = Math.round(res.totalDistanceM);
                segRemainDistM = 35;
            }
        }
        sendSimulatorState();
        broadcastAmapNavigation();
        updateHUD();
    }

    private void toggleNavigation() {
        isNavigating = !isNavigating;
        sendSimulatorState();
        broadcastAmapNavigation();
        updateHUD();
    }

    private void setGear(String gear) {
        currentGear = gear.toUpperCase(Locale.ROOT);
        if (gearLinkage != null) {
            gearLinkage.onGear(currentGear, "R".equals(currentGear) ? "reverse" : "none");
        }
        sendSimulatorState();
        updateHUD();
    }

    private void updateHUD() {
        if (speedNumberView != null) speedNumberView.setText(String.valueOf(actualSpeedKmh));

        for (int i = 0; i < 4; i++) {
            Button b = gearButtons[i];
            if (b == null) continue;
            boolean active = gearNames[i].equals(currentGear);
            int activeColor = "R".equals(gearNames[i]) ? COLOR_RED :
                    "D".equals(gearNames[i]) ? COLOR_CYAN :
                            "P".equals(gearNames[i]) ? COLOR_AMBER : Color.rgb(107, 114, 128);
            b.setBackground(roundRect(active ? activeColor : Color.rgb(31, 41, 55), 8, 0, 0));
            b.setTextColor(active ? Color.WHITE : Color.rgb(156, 163, 175));
        }

        if (btnNavToggle != null) {
            btnNavToggle.setText(isNavigating ? "暂停导航" : "开始导航");
            btnNavToggle.setBackground(roundRect(isNavigating ? COLOR_AMBER : COLOR_GREEN, 12, 0, 0));
        }

        if (hudMainText != null) {
            hudMainText.setText(String.format(Locale.US, "前方 %d 米 %s", segRemainDistM,
                    (segRemainDistM <= 15) ? "路口请减速" : "沿当前车道行驶"));
        }
        if (hudSubText != null) {
            hudSubText.setText(String.format(Locale.US, "前往 %s · 剩余 %d 米 · 限速 %d km/h · %s",
                    targetDestName, routeRemainDistM, currentSpeedLimitKmh, isNavigating ? "自动驾驶循迹中" : "已暂停"));
        }
        if (hudProgressBar != null) {
            int progress = Math.max(0, Math.min(100, 100 - (routeRemainDistM * 100 / Math.max(1, routeRemainDistM + 50))));
            hudProgressBar.setProgress(progress);
        }

        // 仅在真实触发 AEB 制动时显示警报
        if (aebAlertBanner != null) {
            aebAlertBanner.setVisibility(aebActive ? View.VISIBLE : View.GONE);
        }
    }

    private void broadcastAmapNavigation() {
        Intent intent = new Intent(GatewayService.AMAP_ACTION);
        intent.setPackage(getPackageName());
        intent.putExtra("KEY_TYPE", 10001);
        intent.putExtra("ICON", navIcon);
        intent.putExtra("NEW_ICON", navIcon);
        intent.putExtra("SEG_REMAIN_DIS_AUTO", segRemainDistM + "米");
        intent.putExtra("ROUTE_REMAIN_DIS_AUTO", routeRemainDistM + "米");
        int remainSec = (actualSpeedKmh > 0) ? (routeRemainDistM * 3600 / (actualSpeedKmh * 1000)) : 30;
        intent.putExtra("ROUTE_REMAIN_TIME_AUTO", remainSec + "秒");
        intent.putExtra("CUR_SPEED", actualSpeedKmh);
        intent.putExtra("LIMITED_SPEED", currentSpeedLimitKmh);
        intent.putExtra("MANEUVER_NAME", maneuverText);
        sendBroadcast(intent);
    }

    private void sendSimulatorState() {
        Intent intent = new Intent(GatewayService.SIMULATOR_GEAR);
        intent.setPackage(getPackageName());
        intent.putExtra("gear", currentGear);
        intent.putExtra("camera", "R".equals(currentGear) ? "reverse" : "none");
        intent.putExtra("target_speed_kmh", targetSpeedKmh);
        intent.putExtra("actual_speed_kmh", actualSpeedKmh);
        intent.putExtra("destination_x", targetDestX);
        intent.putExtra("destination_y", targetDestY);
        intent.putExtra("destination_name", targetDestName);
        intent.putExtra("simulation_running", isNavigating);
        intent.putExtra("segment_remaining_distance_m", segRemainDistM);
        intent.putExtra("route_remaining_distance_m", routeRemainDistM);
        sendBroadcast(intent);
    }

    private GradientDrawable roundRect(int color, int radiusDp, int strokeColor, int strokeDp) {
        GradientDrawable d = new GradientDrawable();
        d.setColor(color);
        d.setCornerRadius(dp(radiusDp));
        if (strokeDp > 0) d.setStroke(dp(strokeDp), strokeColor);
        return d;
    }

    private GradientDrawable roundStroke(int fill, int stroke, int radiusDp, int strokeDp) {
        GradientDrawable d = new GradientDrawable();
        d.setColor(fill);
        d.setCornerRadius(dp(radiusDp));
        d.setStroke(dp(strokeDp), stroke);
        return d;
    }

    private LinearLayout.LayoutParams margin(int l, int t, int r, int b) {
        LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(-1, -2);
        lp.setMargins(l, t, r, b);
        return lp;
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

    private int dp(float val) {
        return Math.round(val * getResources().getDisplayMetrics().density);
    }
}
