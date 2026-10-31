package cn.embedded.amapgateway;

import android.content.Intent;
import android.os.Bundle;

import java.io.ByteArrayOutputStream;
import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import android.util.Log;

final class NavFrame {
    private static final String TAG = "AmapUartGateway";
    static final int PAYLOAD_LENGTH = 22;
    private static final Pattern NUMBER = Pattern.compile("-?\\d+(?:\\.\\d+)?");

    static byte[] fromIntent(Intent intent, int sequence) {
        Bundle extras = intent.getExtras();
        int keyType = intValue(intent, extras, "KEY_TYPE");
        if (keyType != 10001 && keyType != 12205) {
            Log.w(TAG, "忽略非导航广播 KEY_TYPE=" + keyType + ", extras="
                    + (extras == null ? "{}" : extras));
            return null;
        }

        int icon = intent.getIntExtra("NEW_ICON", 0);
        if (icon == 0) icon = intent.getIntExtra("ICON", 0);
        long segmentDistance = distanceMeters(first(extras, "SEG_REMAIN_DIS_AUTO", "SEG_REMAIN_DIS"));
        long routeDistance = distanceMeters(first(extras, "ROUTE_REMAIN_DIS_AUTO", "ROUTE_REMAIN_DIS"));
        long remainSeconds = durationSeconds(first(extras, "ROUTE_REMAIN_TIME_AUTO", "ROUTE_REMAIN_TIME"));
        int currentSpeed = value(intent, extras, "CUR_SPEED");
        int limitedSpeed = value(intent, extras, "LIMITED_SPEED");

        ByteArrayOutputStream body = new ByteArrayOutputStream();
        put16(body, 1);                 // protocol version
        put16(body, 1);                 // navigation information
        put16(body, sequence);
        put16(body, PAYLOAD_LENGTH);
        put16(body, keyType);
        put16(body, icon);
        put32(body, segmentDistance);
        put32(body, routeDistance);
        put32(body, remainSeconds);
        put16(body, currentSpeed);
        put16(body, limitedSpeed);
        body.write(icon == 0 ? 0 : 1); // 1=turn guidance active, 0=cruise/no turn icon
        body.write(0);                 // reserved

        byte[] protectedBytes = body.toByteArray();
        ByteArrayOutputStream frame = new ByteArrayOutputStream();
        frame.write(0x5A);
        frame.write(0xA5);
        frame.write(protectedBytes, 0, protectedBytes.length);
        int crc = crc16(protectedBytes);
        put16(frame, crc);
        frame.write(0xBB);
        frame.write(0xBB);
        return frame.toByteArray();
    }

    private static Object first(Bundle extras, String... keys) {
        if (extras == null) return null;
        for (String key : keys) if (extras.containsKey(key)) return extras.get(key);
        return null;
    }

    private static int value(Intent intent, Bundle extras, String key) {
        Object raw = first(extras, key);
        if (raw == null) return Math.max(0, intent.getIntExtra(key, 0));
        return (int) Math.max(0, Math.min(65535, number(raw)));
    }

    private static int intValue(Intent intent, Bundle extras, String key) {
        Object raw = first(extras, key);
        if (raw != null) return (int) Math.max(Integer.MIN_VALUE, Math.min(Integer.MAX_VALUE, number(raw)));
        return intent.getIntExtra(key, 0);
    }

    private static long distanceMeters(Object raw) {
        if (raw == null) return 0;
        String text = String.valueOf(raw);
        double n = number(raw);
        if (text.contains("公里") || text.toLowerCase(Locale.ROOT).contains("km")) n *= 1000;
        return clamp32(Math.round(n));
    }

    private static long durationSeconds(Object raw) {
        if (raw == null) return 0;
        String text = String.valueOf(raw);
        double n = number(raw);
        if (text.contains("小时")) n *= 3600;
        else if (text.contains("分钟") || text.toLowerCase(Locale.ROOT).contains("min")) n *= 60;
        return clamp32(Math.round(n));
    }

    private static double number(Object raw) {
        if (raw instanceof Number) return ((Number) raw).doubleValue();
        Matcher matcher = NUMBER.matcher(String.valueOf(raw));
        return matcher.find() ? Double.parseDouble(matcher.group()) : 0;
    }

    private static long clamp32(long value) {
        return Math.max(0, Math.min(0xFFFFFFFFL, value));
    }

    private static int crc16(byte[] bytes) {
        int crc = 0xFFFF;
        for (byte value : bytes) {
            crc ^= (value & 0xFF) << 8;
            for (int i = 0; i < 8; i++) crc = (crc & 0x8000) != 0 ? ((crc << 1) ^ 0x1021) & 0xFFFF : (crc << 1) & 0xFFFF;
        }
        return crc;
    }

    private static void put16(ByteArrayOutputStream out, long value) {
        out.write((int) value & 0xFF);
        out.write((int) (value >>> 8) & 0xFF);
    }

    private static void put32(ByteArrayOutputStream out, long value) {
        out.write((int) value & 0xFF);
        out.write((int) (value >>> 8) & 0xFF);
        out.write((int) (value >>> 16) & 0xFF);
        out.write((int) (value >>> 24) & 0xFF);
    }
}
