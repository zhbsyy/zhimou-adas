package cn.embedded.amapgateway;

import android.animation.ValueAnimator;
import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.DashPathEffect;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.PointF;
import android.graphics.RectF;
import android.util.AttributeSet;
import android.view.GestureDetector;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;
import android.view.View;
import android.view.animation.LinearInterpolator;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * 原生 Town10HD 小镇座舱数字孪生地图画布。
 * 具备类高德车机导航 UI：高精道路底图、车道虚线、动态导航轨迹、
 * 自车平滑平移旋转、行人预警走廊与触控选点导航。
 */
public final class CockpitMapView extends View {

    // 调色板：高德夜间 / 极客深色科技座舱风格
    private static final int COLOR_BG = Color.rgb(13, 19, 31);
    private static final int COLOR_GRID = Color.rgb(22, 32, 51);
    private static final int COLOR_ROAD_BODY = Color.rgb(30, 38, 56);
    private static final int COLOR_ROAD_JUNC = Color.rgb(36, 46, 68);
    private static final int COLOR_ROAD_BORDER = Color.rgb(46, 58, 82);
    private static final int COLOR_LANE_DASH = Color.rgb(60, 77, 108);

    private static final int COLOR_NAV_GLOW = Color.argb(80, 0, 230, 118);
    private static final int COLOR_NAV_LINE = Color.rgb(0, 230, 118);
    private static final int COLOR_NAV_ARROW = Color.WHITE;

    private static final int COLOR_VEHICLE_BODY = Color.rgb(0, 176, 255);
    private static final int COLOR_VEHICLE_PULSE = Color.argb(60, 0, 176, 255);

    private static final int COLOR_PED_HAZARD = Color.argb(90, 255, 23, 68);
    private static final int COLOR_PED_ICON = Color.rgb(255, 23, 68);

    public interface OnLocationSelectedListener {
        void onLocationSelected(float worldX, float worldY, String description);
    }

    private TownMapData mapData;
    private OnLocationSelectedListener selectionListener;

    // 视口与变换
    private float scale = 4.8f; // 像素/米
    private float cameraX = -4.0f; // 当前视口中心的世界坐标 X (默认都会立交)
    private float cameraY = 24.7f; // 当前视口中心的世界坐标 Y
    private boolean followVehicle = true;

    // 自车平滑位姿 (默认起点：都会立交枢纽)
    private float vehX = -4.0f;
    private float vehY = 24.7f;
    private float vehYawDeg = 0.2f;
    private float targetVehX = -4.0f;
    private float targetVehY = 24.7f;
    private float targetVehYawDeg = 0.2f;

    // 行人预警与 AEB 状态
    private boolean pedestrianDetected = false;
    private float pedestrianDistM = 0.0f;
    private float pedestrianLateralM = 0.0f;
    private boolean aebActive = false;

    // 当前导航路线路点 [x, y, yaw]
    private final List<float[]> currentPath = new ArrayList<>();
    // 目的地标记
    private PointF destinationPoint = null;
    private String destinationName = "";

    // 绘图工具
    private final Paint paintRoad = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint paintBorder = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint paintCenterLine = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint paintNavGlow = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint paintNav = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint paintVehicle = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint paintText = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint paintHazard = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Path tempPath = new Path();

    // 动画呼吸值
    private float pulseRadius = 0.0f;
    private ValueAnimator pulseAnimator;

    // 手势识别
    private ScaleGestureDetector scaleDetector;
    private GestureDetector gestureDetector;

    public CockpitMapView(Context context) {
        super(context);
        init(context);
    }

    public CockpitMapView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init(context);
    }

    private void init(Context context) {
        paintRoad.setStyle(Paint.Style.STROKE);
        paintRoad.setStrokeCap(Paint.Cap.ROUND);
        paintRoad.setStrokeJoin(Paint.Join.ROUND);

        paintBorder.setStyle(Paint.Style.STROKE);
        paintBorder.setStrokeCap(Paint.Cap.ROUND);
        paintBorder.setColor(COLOR_ROAD_BORDER);
        paintBorder.setStrokeWidth(dp(1.2f));

        paintCenterLine.setStyle(Paint.Style.STROKE);
        paintCenterLine.setColor(COLOR_LANE_DASH);
        paintCenterLine.setStrokeWidth(dp(1.0f));
        paintCenterLine.setPathEffect(new DashPathEffect(new float[]{dp(6), dp(4)}, 0));

        paintNavGlow.setStyle(Paint.Style.STROKE);
        paintNavGlow.setStrokeCap(Paint.Cap.ROUND);
        paintNavGlow.setStrokeJoin(Paint.Join.ROUND);
        paintNavGlow.setColor(COLOR_NAV_GLOW);

        paintNav.setStyle(Paint.Style.STROKE);
        paintNav.setStrokeCap(Paint.Cap.ROUND);
        paintNav.setStrokeJoin(Paint.Join.ROUND);
        paintNav.setColor(COLOR_NAV_LINE);

        paintVehicle.setStyle(Paint.Style.FILL);
        paintText.setTextAlign(Paint.Align.CENTER);

        paintHazard.setStyle(Paint.Style.FILL);
        paintHazard.setColor(COLOR_PED_HAZARD);

        scaleDetector = new ScaleGestureDetector(context, new ScaleGestureDetector.SimpleOnScaleGestureListener() {
            @Override
            public boolean onScale(ScaleGestureDetector detector) {
                scale *= detector.getScaleFactor();
                scale = Math.max(1.5f, Math.min(18.0f, scale));
                invalidate();
                return true;
            }
        });

        gestureDetector = new GestureDetector(context, new GestureDetector.SimpleOnGestureListener() {
            @Override
            public boolean onScroll(MotionEvent e1, MotionEvent e2, float distanceX, float distanceY) {
                followVehicle = false;
                cameraX += distanceX / scale;
                cameraY += distanceY / scale;
                invalidate();
                return true;
            }

            @Override
            public boolean onSingleTapConfirmed(MotionEvent e) {
                handleTap(e.getX(), e.getY());
                return true;
            }
        });

        pulseAnimator = ValueAnimator.ofFloat(0.0f, 1.0f);
        pulseAnimator.setDuration(1200);
        pulseAnimator.setRepeatCount(ValueAnimator.INFINITE);
        pulseAnimator.setInterpolator(new LinearInterpolator());
        pulseAnimator.addUpdateListener(animation -> {
            pulseRadius = (float) animation.getAnimatedValue();
            // 在每一拍平滑插值移动自车
            vehX += (targetVehX - vehX) * 0.25f;
            vehY += (targetVehY - vehY) * 0.25f;
            float diffYaw = (targetVehYawDeg - vehYawDeg + 180) % 360 - 180;
            vehYawDeg += diffYaw * 0.25f;

            if (followVehicle) {
                cameraX += (vehX - cameraX) * 0.2f;
                cameraY += (vehY - cameraY) * 0.2f;
            }
            invalidate();
        });
        pulseAnimator.start();
    }

    public void setMapData(TownMapData data) {
        this.mapData = data;
        invalidate();
    }

    public void setOnLocationSelectedListener(OnLocationSelectedListener listener) {
        this.selectionListener = listener;
    }

    public void setFollowVehicle(boolean follow) {
        this.followVehicle = follow;
        if (follow) {
            cameraX = vehX;
            cameraY = vehY;
        }
        invalidate();
    }

    public boolean isFollowVehicle() {
        return followVehicle;
    }

    public void zoomIn() {
        scale = Math.min(18.0f, scale * 1.3f);
        invalidate();
    }

    public void zoomOut() {
        scale = Math.max(1.5f, scale / 1.3f);
        invalidate();
    }

    /** 更新自车位姿（支持插值平滑移动） */
    public void updateVehiclePose(float x, float y, float yawDeg) {
        this.targetVehX = x;
        this.targetVehY = y;
        this.targetVehYawDeg = yawDeg;
        if (Math.hypot(targetVehX - vehX, targetVehY - vehY) > 20.0f) {
            // 距离过大时直接同步瞬移
            this.vehX = x;
            this.vehY = y;
            this.vehYawDeg = yawDeg;
        }
        if (followVehicle) {
            cameraX = vehX;
            cameraY = vehY;
        }
        invalidate();
    }

    /** 更新行人检测与 AEB 制动警报 */
    public void updatePedestrianAlert(boolean detected, float distM, float lateralM, boolean aeb) {
        this.pedestrianDetected = detected;
        this.pedestrianDistM = distM;
        this.pedestrianLateralM = lateralM;
        this.aebActive = aeb;
        invalidate();
    }

    /** 设置当前激活的高德导航路径点序列 */
    public void setNavigationPath(List<float[]> path) {
        currentPath.clear();
        if (path != null) {
            currentPath.addAll(path);
        }
        invalidate();
    }

    public List<float[]> getNavigationPath() {
        return currentPath;
    }

    /** 设置目的地 */
    public void setDestination(float x, float y, String name) {
        this.destinationPoint = new PointF(x, y);
        this.destinationName = name;
        invalidate();
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        boolean handled = scaleDetector.onTouchEvent(event);
        handled = gestureDetector.onTouchEvent(event) || handled;
        return handled || super.onTouchEvent(event);
    }

    private void handleTap(float sx, float sy) {
        float wx = toWorldX(sx);
        float wy = toWorldY(sy);
        if (mapData != null) {
            // 检查是否点中了预设 POI
            for (TownMapData.POI poi : mapData.pois) {
                float dx = poi.x - wx;
                float dy = poi.y - wy;
                if (Math.hypot(dx, dy) < 12.0) {
                    setDestination(poi.x, poi.y, poi.name);
                    if (selectionListener != null) {
                        selectionListener.onLocationSelected(poi.x, poi.y, poi.name);
                    }
                    return;
                }
            }

            // 查找最近道路点
            PointF nearest = mapData.findNearestRoadPoint(wx, wy);
            if (nearest != null) {
                String desc = String.format(Locale.US, "路网坐标 (%.1f, %.1f)", nearest.x, nearest.y);
                setDestination(nearest.x, nearest.y, desc);
                if (selectionListener != null) {
                    selectionListener.onLocationSelected(nearest.x, nearest.y, desc);
                }
            }
        }
    }

    // 坐标转换公式
    private float toScreenX(float worldX) {
        return getWidth() / 2.0f + (worldX - cameraX) * scale;
    }

    private float toScreenY(float worldY) {
        return getHeight() / 2.0f + (worldY - cameraY) * scale;
    }

    private float toWorldX(float screenX) {
        return (screenX - getWidth() / 2.0f) / scale + cameraX;
    }

    private float toWorldY(float screenY) {
        return (screenY - getHeight() / 2.0f) / scale + cameraY;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        int w = getWidth();
        int h = getHeight();
        if (w == 0 || h == 0) return;

        // 1. 背景科技夜间底色
        canvas.drawColor(COLOR_BG);

        // 2. 绘制参考网格 (50m 间隔)
        drawGrid(canvas, w, h);

        if (mapData != null) {
            // 3. 绘制道路网路面
            drawRoads(canvas);

            // 4. 绘制预设地标 POI
            drawPois(canvas);
        }

        // 5. 绘制当前导航规划高亮路径
        drawNavigationPath(canvas);

        // 6. 绘制行人横穿危险走廊与 AEB 警示
        drawPedestrianHazard(canvas);

        // 7. 绘制目的地 Flag / 图钉
        drawDestination(canvas);

        // 8. 绘制自车光标与航向
        drawEgoVehicle(canvas);
    }

    private void drawGrid(Canvas canvas, int w, int h) {
        Paint p = new Paint();
        p.setColor(COLOR_GRID);
        p.setStrokeWidth(1.0f);

        float gridStepM = 40.0f;
        float startX = (float) Math.floor(toWorldX(0) / gridStepM) * gridStepM;
        float endX = (float) Math.ceil(toWorldX(w) / gridStepM) * gridStepM;
        float startY = (float) Math.floor(toWorldY(0) / gridStepM) * gridStepM;
        float endY = (float) Math.ceil(toWorldY(h) / gridStepM) * gridStepM;

        for (float x = startX; x <= endX; x += gridStepM) {
            float sx = toScreenX(x);
            canvas.drawLine(sx, 0, sx, h, p);
        }
        for (float y = startY; y <= endY; y += gridStepM) {
            float sy = toScreenY(y);
            canvas.drawLine(0, sy, w, sy, p);
        }
    }

    private void drawRoads(Canvas canvas) {
        for (TownMapData.Lane lane : mapData.lanes) {
            if (lane.points.size() < 2) continue;

            tempPath.reset();
            PointF first = lane.points.get(0);
            tempPath.moveTo(toScreenX(first.x), toScreenY(first.y));
            for (int i = 1; i < lane.points.size(); i++) {
                PointF pt = lane.points.get(i);
                tempPath.lineTo(toScreenX(pt.x), toScreenY(pt.y));
            }

            // 绘制沥青路面
            paintRoad.setColor(lane.isJunction ? COLOR_ROAD_JUNC : COLOR_ROAD_BODY);
            paintRoad.setStrokeWidth(Math.max(dp(4), lane.width * scale));
            canvas.drawPath(tempPath, paintRoad);

            // 绘制路缘线
            canvas.drawPath(tempPath, paintBorder);

            // 绘制车道虚线中线
            canvas.drawPath(tempPath, paintCenterLine);
        }
    }

    private void drawPois(Canvas canvas) {
        paintText.setTextSize(dp(11));
        Paint bgPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        bgPaint.setColor(Color.argb(180, 15, 23, 42));

        for (TownMapData.POI poi : mapData.pois) {
            float sx = toScreenX(poi.x);
            float sy = toScreenY(poi.y);

            // 如果在视口外太远则跳过
            if (sx < -100 || sx > getWidth() + 100 || sy < -100 || sy > getHeight() + 100) continue;

            boolean isStart = "start".equals(poi.type);
            int circleColor = isStart ? Color.rgb(0, 230, 118) : Color.rgb(255, 179, 0);

            // POI 外光环
            Paint ring = new Paint(Paint.ANTI_ALIAS_FLAG);
            ring.setStyle(Paint.Style.STROKE);
            ring.setColor(circleColor);
            ring.setStrokeWidth(dp(1.8f));
            canvas.drawCircle(sx, sy, dp(7), ring);

            Paint dot = new Paint(Paint.ANTI_ALIAS_FLAG);
            dot.setColor(circleColor);
            canvas.drawCircle(sx, sy, dp(3.5f), dot);

            // POI 标签胶囊底板
            String label = poi.name;
            float textWidth = paintText.measureText(label);
            RectF pill = new RectF(sx - textWidth / 2f - dp(6), sy + dp(10), sx + textWidth / 2f + dp(6), sy + dp(24));
            canvas.drawRoundRect(pill, dp(6), dp(6), bgPaint);

            paintText.setColor(Color.rgb(226, 232, 240));
            canvas.drawText(label, sx, sy + dp(20), paintText);
        }
    }

    private void drawNavigationPath(Canvas canvas) {
        if (currentPath.size() < 2) return;

        // 1. 查找自车在规划路线上的最近投影匹配点索引
        int matchIdx = 0;
        float minD = Float.MAX_VALUE;
        for (int i = 0; i < currentPath.size(); i++) {
            float[] wp = currentPath.get(i);
            float d = (float) Math.hypot(wp[0] - vehX, wp[1] - vehY);
            if (d < minD) {
                minD = d;
                matchIdx = i;
            }
        }

        // 选取自车前方的第一个导航路点
        int startIdx = Math.min(matchIdx + 1, currentPath.size() - 1);
        if (startIdx >= currentPath.size()) return;

        tempPath.reset();
        // 从自车车头中心点直接平滑引出轨迹，使轨迹无论变道与否始终紧密贴合自车
        tempPath.moveTo(toScreenX(vehX), toScreenY(vehY));
        for (int i = startIdx; i < currentPath.size(); i++) {
            float[] wp = currentPath.get(i);
            tempPath.lineTo(toScreenX(wp[0]), toScreenY(wp[1]));
        }

        // 1. 发光外轮廓 (Neon Glow)
        paintNavGlow.setStrokeWidth(dp(14));
        canvas.drawPath(tempPath, paintNavGlow);

        // 2. 荧光主轨迹线
        paintNav.setStrokeWidth(dp(5.5f));
        canvas.drawPath(tempPath, paintNav);

        // 3. 沿路线绘制指示箭头 (每隔 7 个路点绘制一个箭头)
        Paint arrowPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        arrowPaint.setColor(COLOR_NAV_ARROW);
        arrowPaint.setStyle(Paint.Style.FILL);

        for (int i = startIdx + 3; i < currentPath.size() - 2; i += 7) {
            float[] p1 = currentPath.get(i);
            float[] p2 = currentPath.get(i + 1);
            float sx1 = toScreenX(p1[0]);
            float sy1 = toScreenY(p1[1]);
            float sx2 = toScreenX(p2[0]);
            float sy2 = toScreenY(p2[1]);
            float angle = (float) Math.toDegrees(Math.atan2(sy2 - sy1, sx2 - sx1));

            canvas.save();
            canvas.translate(sx1, sy1);
            canvas.rotate(angle);
            Path arr = new Path();
            arr.moveTo(dp(4), 0);
            arr.lineTo(-dp(3), -dp(3));
            arr.lineTo(-dp(1), 0);
            arr.lineTo(-dp(3), dp(3));
            arr.close();
            canvas.drawPath(arr, arrowPaint);
            canvas.restore();
        }
    }

    private void drawPedestrianHazard(Canvas canvas) {
        // 严格遵循用户要求：未触发视觉 AEB 介入时绝不显示伪虚警
        if (!aebActive) return;

        // 根据自车位置与前向距离推算行人的大概屏幕坐标
        float rad = (float) Math.toRadians(vehYawDeg);
        float fwdX = (float) Math.cos(rad);
        float fwdY = (float) Math.sin(rad);
        float rgtX = -fwdY;
        float rgtY = fwdX;

        float pedDist = Math.max(2.0f, pedestrianDistM);
        float pedWorldX = vehX + fwdX * pedDist + rgtX * pedestrianLateralM;
        float pedWorldY = vehY + fwdY * pedDist + rgtY * pedestrianLateralM;

        float pedSx = toScreenX(pedWorldX);
        float pedSy = toScreenY(pedWorldY);
        float vehSx = toScreenX(vehX);
        float vehSy = toScreenY(vehY);

        // 1. 绘制危险警戒光锥扇区
        Path cone = new Path();
        cone.moveTo(vehSx, vehSy);
        cone.lineTo(pedSx - dp(20), pedSy);
        cone.lineTo(pedSx + dp(20), pedSy);
        cone.close();
        canvas.drawPath(cone, paintHazard);

        // 2. 绘制行人警报标记
        float pulse = (float) Math.sin(System.currentTimeMillis() / 150.0) * 0.5f + 0.5f;
        Paint pedPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        pedPaint.setColor(COLOR_PED_ICON);
        pedPaint.setStyle(Paint.Style.FILL);
        pedPaint.setAlpha((int) (160 + 95 * pulse));

        canvas.drawCircle(pedSx, pedSy, dp(10 + 4 * pulse), pedPaint);

        Paint ring = new Paint(Paint.ANTI_ALIAS_FLAG);
        ring.setStyle(Paint.Style.STROKE);
        ring.setColor(Color.WHITE);
        ring.setStrokeWidth(dp(2));
        canvas.drawCircle(pedSx, pedSy, dp(10), ring);

        // 3. 标签
        Paint txtP = new Paint(Paint.ANTI_ALIAS_FLAG);
        txtP.setColor(Color.WHITE);
        txtP.setTextSize(dp(11));
        txtP.setFakeBoldText(true);
        txtP.setTextAlign(Paint.Align.CENTER);

        String alertText = aebActive
                ? String.format(Locale.US, "AEB 紧急制动！%.1fm", pedDist)
                : String.format(Locale.US, "横穿行人预警 %.1fm", pedDist);

        Paint pillBg = new Paint(Paint.ANTI_ALIAS_FLAG);
        pillBg.setColor(Color.rgb(211, 47, 47));
        float tW = txtP.measureText(alertText);
        RectF pill = new RectF(pedSx - tW / 2f - dp(8), pedSy - dp(32), pedSx + tW / 2f + dp(8), pedSy - dp(14));
        canvas.drawRoundRect(pill, dp(6), dp(6), pillBg);
        canvas.drawText(alertText, pedSx, pedSy - dp(20), txtP);
    }

    private void drawDestination(Canvas canvas) {
        if (destinationPoint == null) return;
        float sx = toScreenX(destinationPoint.x);
        float sy = toScreenY(destinationPoint.y);

        // 终点发光外圈
        Paint p = new Paint(Paint.ANTI_ALIAS_FLAG);
        p.setColor(Color.rgb(255, 61, 0));
        p.setStyle(Paint.Style.FILL);
        canvas.drawCircle(sx, sy, dp(7), p);

        p.setColor(Color.WHITE);
        canvas.drawCircle(sx, sy, dp(3), p);

        // 终点旗标卡片
        if (!destinationName.isEmpty()) {
            Paint bg = new Paint(Paint.ANTI_ALIAS_FLAG);
            bg.setColor(Color.argb(220, 220, 38, 38));
            Paint tp = new Paint(Paint.ANTI_ALIAS_FLAG);
            tp.setColor(Color.WHITE);
            tp.setTextSize(dp(11));
            tp.setTextAlign(Paint.Align.CENTER);

            float tw = tp.measureText("目的地: " + destinationName);
            RectF pill = new RectF(sx - tw / 2f - dp(8), sy - dp(30), sx + tw / 2f + dp(8), sy - dp(12));
            canvas.drawRoundRect(pill, dp(6), dp(6), bg);
            canvas.drawText("目的地: " + destinationName, sx, sy - dp(18), tp);
        }
    }

    private void drawEgoVehicle(Canvas canvas) {
        float sx = toScreenX(vehX);
        float sy = toScreenY(vehY);

        // 1. 脉冲呼吸发光圆环
        Paint pulsePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        pulsePaint.setColor(COLOR_VEHICLE_PULSE);
        pulsePaint.setStyle(Paint.Style.FILL);
        float maxR = dp(24);
        float curR = dp(10) + maxR * pulseRadius;
        pulsePaint.setAlpha((int) ((1.0f - pulseRadius) * 120));
        canvas.drawCircle(sx, sy, curR, pulsePaint);

        // 2. 自车立体高德箭头
        canvas.save();
        canvas.translate(sx, sy);
        // CARLA 中 yaw 旋转：在标准屏幕坐标系中旋转
        canvas.rotate(vehYawDeg);

        // 车标阴影
        Paint shadow = new Paint(Paint.ANTI_ALIAS_FLAG);
        shadow.setColor(Color.argb(80, 0, 0, 0));
        Path shp = new Path();
        shp.moveTo(dp(14), 0);
        shp.lineTo(-dp(10), -dp(8));
        shp.lineTo(-dp(5), 0);
        shp.lineTo(-dp(10), dp(8));
        shp.close();
        canvas.drawPath(shp, shadow);

        // 车标主体 (青蓝高光渐变科技箭头)
        paintVehicle.setColor(COLOR_VEHICLE_BODY);
        canvas.drawPath(shp, paintVehicle);

        // 车头尖角白色高光描边
        Paint edge = new Paint(Paint.ANTI_ALIAS_FLAG);
        edge.setStyle(Paint.Style.STROKE);
        edge.setColor(Color.WHITE);
        edge.setStrokeWidth(dp(1.5f));
        canvas.drawPath(shp, edge);

        // 车中心白色小圆点
        Paint centerDot = new Paint(Paint.ANTI_ALIAS_FLAG);
        centerDot.setColor(Color.WHITE);
        canvas.drawCircle(0, 0, dp(2.2f), centerDot);

        canvas.restore();
    }

    private int dp(float val) {
        return Math.round(val * getResources().getDisplayMetrics().density);
    }
}
