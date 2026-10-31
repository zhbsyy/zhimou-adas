package cn.embedded.amapgateway;

import android.content.Context;
import android.graphics.PointF;
import android.util.Log;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;

/**
 * Town10HD 原生小镇高精道路矢量数据模型与自主寻路规划器。
 * 支持根据当前自车位置和点击的目的地，通过拓扑图运行 Dijkstra 算法动态实时规划最短路径。
 */
public final class TownMapData {
    private static final String TAG = "TownMapData";

    public static final class Lane {
        public final int roadId;
        public final int laneId;
        public final float width;
        public final boolean isJunction;
        public final List<PointF> points;

        public Lane(int roadId, int laneId, float width, boolean isJunction, List<PointF> points) {
            this.roadId = roadId;
            this.laneId = laneId;
            this.width = width;
            this.isJunction = isJunction;
            this.points = points;
        }
    }

    public static final class POI {
        public final String id;
        public final String name;
        public final float x;
        public final float y;
        public final String type;

        public POI(String id, String name, float x, float y, String type) {
            this.id = id;
            this.name = name;
            this.x = x;
            this.y = y;
            this.type = type;
        }
    }

    public static final class GraphNode {
        public final int id;
        public final float x;
        public final float y;

        public GraphNode(int id, float x, float y) {
            this.id = id;
            this.x = x;
            this.y = y;
        }
    }

    public static final class GraphEdge {
        public final int u;
        public final int v;
        public final float length;
        public final int type;
        public final List<float[]> path;

        public GraphEdge(int u, int v, float length, int type, List<float[]> path) {
            this.u = u;
            this.v = v;
            this.length = length;
            this.type = type;
            this.path = path;
        }
    }

    public static final class RouteResult {
        public final List<float[]> waypoints;
        public final float totalDistanceM;
        public final int initialTurnIcon;

        public RouteResult(List<float[]> waypoints, float totalDistanceM, int initialTurnIcon) {
            this.waypoints = waypoints;
            this.totalDistanceM = totalDistanceM;
            this.initialTurnIcon = initialTurnIcon;
        }
    }

    public final List<Lane> lanes = new ArrayList<>();
    public final List<POI> pois = new ArrayList<>();
    public final Map<Integer, GraphNode> nodes = new HashMap<>();
    public final Map<Integer, List<GraphEdge>> adj = new HashMap<>();
    public final Map<String, int[]> roadToEdge = new HashMap<>();

    public float minX = -115f;
    public float maxX = 110f;
    public float minY = -70f;
    public float maxY = 142f;

    // 默认起点：都会立交枢纽
    public float defaultSpawnX = -4.0f;
    public float defaultSpawnY = 24.7f;
    public float defaultSpawnYaw = 0.2f;

    public static TownMapData loadFromAssets(Context context) {
        TownMapData data = new TownMapData();
        try (InputStream is = context.getAssets().open("town10_map.json")) {
            int size = is.available();
            byte[] buffer = new byte[size];
            is.read(buffer);
            String jsonStr = new String(buffer, StandardCharsets.UTF_8);
            JSONObject root = new JSONObject(jsonStr);

            if (root.has("bounds")) {
                JSONObject b = root.getJSONObject("bounds");
                data.minX = (float) b.optDouble("min_x", -115.0);
                data.maxX = (float) b.optDouble("max_x", 110.0);
                data.minY = (float) b.optDouble("min_y", -70.0);
                data.maxY = (float) b.optDouble("max_y", 142.0);
            }

            if (root.has("default_spawn")) {
                JSONObject sp = root.getJSONObject("default_spawn");
                data.defaultSpawnX = (float) sp.optDouble("x", -4.0);
                data.defaultSpawnY = (float) sp.optDouble("y", 24.7);
                data.defaultSpawnYaw = (float) sp.optDouble("yaw", 0.2);
            }

            // 1. 道路数据
            JSONArray roadsArr = root.optJSONArray("roads");
            if (roadsArr != null) {
                for (int i = 0; i < roadsArr.length(); i++) {
                    JSONObject r = roadsArr.getJSONObject(i);
                    int rid = r.optInt("road_id", 0);
                    int lid = r.optInt("lane_id", 0);
                    float width = (float) r.optDouble("width", 3.5);
                    boolean isJunc = r.optBoolean("is_junction", false);
                    JSONArray ptsArr = r.optJSONArray("points");
                    List<PointF> pts = new ArrayList<>();
                    if (ptsArr != null) {
                        for (int j = 0; j < ptsArr.length(); j++) {
                            JSONArray pt = ptsArr.getJSONArray(j);
                            pts.add(new PointF((float) pt.getDouble(0), (float) pt.getDouble(1)));
                        }
                    }
                    if (!pts.isEmpty()) {
                        data.lanes.add(new Lane(rid, lid, width, isJunc, pts));
                    }
                }
            }

            // 2. POI 数据
            JSONArray poisArr = root.optJSONArray("pois");
            if (poisArr != null) {
                for (int i = 0; i < poisArr.length(); i++) {
                    JSONObject p = poisArr.getJSONObject(i);
                    data.pois.add(new POI(
                            p.optString("id", ""),
                            p.optString("name", "POI"),
                            (float) p.optDouble("x", 0.0),
                            (float) p.optDouble("y", 0.0),
                            p.optString("type", "landmark")
                    ));
                }
            }

            // 3. 拓扑图数据
            JSONObject graphObj = root.optJSONObject("graph");
            if (graphObj != null) {
                JSONArray nodesArr = graphObj.optJSONArray("nodes");
                if (nodesArr != null) {
                    for (int i = 0; i < nodesArr.length(); i++) {
                        JSONObject n = nodesArr.getJSONObject(i);
                        int id = n.getInt("id");
                        float nx = (float) n.getDouble("x");
                        float ny = (float) n.getDouble("y");
                        data.nodes.put(id, new GraphNode(id, nx, ny));
                    }
                }

                JSONArray edgesArr = graphObj.optJSONArray("edges");
                if (edgesArr != null) {
                    for (int i = 0; i < edgesArr.length(); i++) {
                        JSONObject e = edgesArr.getJSONObject(i);
                        int u = e.getInt("u");
                        int v = e.getInt("v");
                        float len = (float) e.getDouble("len");
                        int type = e.optInt("type", 4);
                        JSONArray pathArr = e.optJSONArray("path");
                        List<float[]> path = new ArrayList<>();
                        if (pathArr != null) {
                            for (int j = 0; j < pathArr.length(); j++) {
                                JSONArray pt = pathArr.getJSONArray(j);
                                path.add(new float[]{
                                        (float) pt.getDouble(0),
                                        (float) pt.getDouble(1),
                                        (float) pt.getDouble(2)
                                });
                            }
                        }
                        float computedLen = 0.0f;
                        if (path.size() >= 2) {
                            for (int k = 0; k < path.size() - 1; k++) {
                                float[] p1 = path.get(k);
                                float[] p2 = path.get(k + 1);
                                computedLen += (float) Math.hypot(p2[0] - p1[0], p2[1] - p1[1]);
                            }
                        } else {
                            computedLen = 6.0f; // 变道合理代价为 6 米
                        }
                        GraphEdge edge = new GraphEdge(u, v, Math.max(1.0f, computedLen), type, path);
                        if (!data.adj.containsKey(u)) {
                            data.adj.put(u, new ArrayList<>());
                        }
                        data.adj.get(u).add(edge);
                    }
                }
            }

            // 4. road_to_edge 索引
            JSONObject r2e = root.optJSONObject("road_to_edge");
            if (r2e != null) {
                for (java.util.Iterator<String> it = r2e.keys(); it.hasNext(); ) {
                    String k = it.next();
                    JSONArray arr = r2e.getJSONArray(k);
                    data.roadToEdge.put(k, new int[]{arr.getInt(0), arr.getInt(1)});
                }
            }

            Log.i(TAG, "成功加载 Town10HD 拓扑数据：" + data.lanes.size() + " 条车道，" + data.nodes.size() + " 个节点，" + data.roadToEdge.size() + " 拓扑边");
        } catch (Exception e) {
            Log.e(TAG, "加载 Town10HD 拓扑地图失败", e);
        }
        return data;
    }

    public PointF findNearestRoadPoint(float worldX, float worldY) {
        PointF best = null;
        float minDstSq = Float.MAX_VALUE;
        for (Lane lane : lanes) {
            for (PointF pt : lane.points) {
                float dx = pt.x - worldX;
                float dy = pt.y - worldY;
                float d2 = dx * dx + dy * dy;
                if (d2 < minDstSq) {
                    minDstSq = d2;
                    best = pt;
                }
            }
        }
        return best;
    }

    private int[] findNearestEdge(float worldX, float worldY) {
        int bestU = -1, bestV = -1;
        float minDstSq = Float.MAX_VALUE;

        // 优先根据最近车道定位
        Lane bestLane = null;
        for (Lane lane : lanes) {
            for (PointF pt : lane.points) {
                float dx = pt.x - worldX;
                float dy = pt.y - worldY;
                float d2 = dx * dx + dy * dy;
                if (d2 < minDstSq) {
                    minDstSq = d2;
                    bestLane = lane;
                }
            }
        }

        if (bestLane != null) {
            String key = bestLane.roadId + "_" + bestLane.laneId;
            if (roadToEdge.containsKey(key)) {
                return roadToEdge.get(key);
            }
        }

        // 回退查找所有 edge path 中最近的点
        minDstSq = Float.MAX_VALUE;
        for (List<GraphEdge> edgeList : adj.values()) {
            for (GraphEdge e : edgeList) {
                for (float[] pt : e.path) {
                    float dx = pt[0] - worldX;
                    float dy = pt[1] - worldY;
                    float d2 = dx * dx + dy * dy;
                    if (d2 < minDstSq) {
                        minDstSq = d2;
                        bestU = e.u;
                        bestV = e.v;
                    }
                }
            }
        }
        return (bestU != -1) ? new int[]{bestU, bestV} : null;
    }

    private GraphEdge findEdge(int u, int v) {
        List<GraphEdge> el = adj.get(u);
        if (el != null) {
            for (GraphEdge e : el) {
                if (e.v == v) return e;
            }
        }
        return null;
    }

    private int findNearestPointIndex(List<float[]> path, float x, float y) {
        if (path == null || path.isEmpty()) return 0;
        int bestI = 0;
        float minD2 = Float.MAX_VALUE;
        for (int i = 0; i < path.size(); i++) {
            float[] p = path.get(i);
            float d2 = (p[0] - x) * (p[0] - x) + (p[1] - y) * (p[1] - y);
            if (d2 < minD2) {
                minD2 = d2;
                bestI = i;
            }
        }
        return bestI;
    }

    private float computePathDistance(List<float[]> path) {
        if (path == null || path.size() < 2) return 0.0f;
        float d = 0.0f;
        for (int i = 0; i < path.size() - 1; i++) {
            float[] p1 = path.get(i);
            float[] p2 = path.get(i + 1);
            d += (float) Math.hypot(p2[0] - p1[0], p2[1] - p1[1]);
        }
        return d;
    }

    /**
     * 自主选择道路的最短路径规划算法 (Dijkstra 搜索拓扑图)
     * 给定起点和终点，严格从自车当前坐标开始规划，绝不往后偏移！
     */
    public RouteResult planRoute(float startX, float startY, float destX, float destY) {
        int[] startEdge = findNearestEdge(startX, startY);
        int[] destEdge = findNearestEdge(destX, destY);

        if (startEdge == null || destEdge == null) {
            return fallbackDirectRoute(startX, startY, destX, destY);
        }

        GraphEdge startE = findEdge(startEdge[0], startEdge[1]);
        GraphEdge destE = findEdge(destEdge[0], destEdge[1]);

        // 1. 若起点和终点位于同一路段边上
        if (startE != null && destE != null && startEdge[0] == destEdge[0] && startEdge[1] == destEdge[1]) {
            int sIdx = findNearestPointIndex(startE.path, startX, startY);
            int dIdx = findNearestPointIndex(startE.path, destX, destY);
            if (sIdx <= dIdx) {
                List<float[]> samePath = new ArrayList<>();
                samePath.add(new float[]{startX, startY, 0.0f});
                for (int i = sIdx + 1; i <= dIdx; i++) {
                    samePath.add(startE.path.get(i));
                }
                samePath.add(new float[]{destX, destY, 0.0f});
                return new RouteResult(samePath, Math.max(5.0f, computePathDistance(samePath)), 1);
            }
        }

        // 2. 自车在当前边上行进，目标节点为当前边的终点节点 startEdge[1]
        int sNode = startEdge[1];
        int dNode = destEdge[0];

        Map<Integer, Float> dist = new HashMap<>();
        Map<Integer, Integer> prevNode = new HashMap<>();
        Map<Integer, GraphEdge> prevEdge = new HashMap<>();

        for (int nid : nodes.keySet()) {
            dist.put(nid, Float.MAX_VALUE);
        }
        dist.put(sNode, 0.0f);

        PriorityQueue<int[]> pq = new PriorityQueue<>(Comparator.comparingDouble(a -> Float.intBitsToFloat(a[1])));
        pq.offer(new int[]{sNode, Float.floatToIntBits(0.0f)});

        while (!pq.isEmpty()) {
            int[] top = pq.poll();
            int u = top[0];
            float d = Float.intBitsToFloat(top[1]);

            if (d > dist.get(u)) continue;
            if (u == dNode) break;

            List<GraphEdge> edges = adj.get(u);
            if (edges != null) {
                for (GraphEdge e : edges) {
                    int v = e.v;
                    float newDist = d + e.length;
                    if (newDist < dist.get(v)) {
                        dist.put(v, newDist);
                        prevNode.put(v, u);
                        prevEdge.put(v, e);
                        pq.offer(new int[]{v, Float.floatToIntBits(newDist)});
                    }
                }
            }
        }

        // 回溯最短边序列
        List<GraphEdge> edgeSeq = new ArrayList<>();
        if (sNode != dNode && prevNode.containsKey(dNode)) {
            int curr = dNode;
            while (curr != sNode && prevNode.containsKey(curr)) {
                edgeSeq.add(prevEdge.get(curr));
                curr = prevNode.get(curr);
            }
            Collections.reverse(edgeSeq);
        } else if (sNode != dNode) {
            // 容错搜索：尝试从当前边始端出发
            int altStart = startEdge[0];
            dist.clear();
            prevNode.clear();
            prevEdge.clear();
            for (int nid : nodes.keySet()) dist.put(nid, Float.MAX_VALUE);
            dist.put(altStart, 0.0f);
            pq.clear();
            pq.offer(new int[]{altStart, Float.floatToIntBits(0.0f)});
            while (!pq.isEmpty()) {
                int[] top = pq.poll();
                int u = top[0];
                float d = Float.intBitsToFloat(top[1]);
                if (d > dist.get(u)) continue;
                if (u == dNode) break;
                List<GraphEdge> edges = adj.get(u);
                if (edges != null) {
                    for (GraphEdge e : edges) {
                        int v = e.v;
                        float newDist = d + e.length;
                        if (newDist < dist.get(v)) {
                            dist.put(v, newDist);
                            prevNode.put(v, u);
                            prevEdge.put(v, e);
                            pq.offer(new int[]{v, Float.floatToIntBits(newDist)});
                        }
                    }
                }
            }
            if (prevNode.containsKey(dNode)) {
                int curr = dNode;
                while (curr != altStart && prevNode.containsKey(curr)) {
                    edgeSeq.add(prevEdge.get(curr));
                    curr = prevNode.get(curr);
                }
                Collections.reverse(edgeSeq);
            } else {
                return fallbackDirectRoute(startX, startY, destX, destY);
            }
        }

        // 3. 构建全量平滑轨迹（首点严格绑定自车当前坐标！）
        List<float[]> fullPath = new ArrayList<>();
        fullPath.add(new float[]{startX, startY, 0.0f});

        // 彻底裁剪：仅提取当前边在自车前方的路点，严禁包含车辆后方的历史路点
        if (startE != null && startE.path != null && !startE.path.isEmpty()) {
            int nearestIdx = findNearestPointIndex(startE.path, startX, startY);
            for (int i = nearestIdx + 1; i < startE.path.size(); i++) {
                fullPath.add(startE.path.get(i));
            }
        }

        int initialIcon = 1;
        for (GraphEdge e : edgeSeq) {
            if (e.path != null) {
                fullPath.addAll(e.path);
            }
            if (initialIcon == 1 && e.type != 4 && e.type != 3) {
                initialIcon = (e.type == 1) ? 2 : ((e.type == 2) ? 3 : 1);
            }
        }

        // 终点裁剪：追加目标边路点直至目标点
        if (destE != null && destE != startE && destE.path != null && !destE.path.isEmpty()) {
            int dIdx = findNearestPointIndex(destE.path, destX, destY);
            for (int i = 0; i <= dIdx; i++) {
                fullPath.add(destE.path.get(i));
            }
        }
        fullPath.add(new float[]{destX, destY, 0.0f});

        float totalDist = computePathDistance(fullPath);
        return new RouteResult(fullPath, Math.max(10.0f, totalDist), initialIcon);
    }

    private RouteResult fallbackDirectRoute(float sx, float sy, float dx, float dy) {
        List<float[]> pts = new ArrayList<>();
        float totalDist = (float) Math.hypot(dx - sx, dy - sy);
        int steps = Math.max(5, (int) (totalDist / 2.0f));
        float yaw = (float) Math.toDegrees(Math.atan2(dy - sy, dx - sx));
        for (int i = 0; i <= steps; i++) {
            float frac = (float) i / steps;
            pts.add(new float[]{sx + (dx - sx) * frac, sy + (dy - sy) * frac, yaw});
        }
        return new RouteResult(pts, totalDist, 1);
    }
}
