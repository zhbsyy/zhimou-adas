package cn.embedded.amapgateway;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.os.Build;

public final class BootReceiver extends BroadcastReceiver {
    @Override public void onReceive(Context context, Intent intent) {
        if (Intent.ACTION_BOOT_COMPLETED.equals(intent.getAction())) {
            android.content.SharedPreferences preferences = context.getSharedPreferences(MainActivity.PREFS, Context.MODE_PRIVATE);
            String configuredPort = preferences.getString("port", "");
            if (configuredPort.isEmpty() || "/dev/ttyS3".equals(configuredPort) || "/dev/ttyAMA10".equals(configuredPort)) configuredPort = "/dev/ttyAMA0";
            preferences.edit()
                    .putString("port", configuredPort)
                    .putInt("baud", context.getSharedPreferences(MainActivity.PREFS, Context.MODE_PRIVATE).getInt("baud", 115200))
                    .putBoolean("enabled", true).apply();
            Intent service = new Intent(context, GatewayService.class);
            if (Build.VERSION.SDK_INT >= 26) context.startForegroundService(service);
            else context.startService(service);

        }
    }
}
