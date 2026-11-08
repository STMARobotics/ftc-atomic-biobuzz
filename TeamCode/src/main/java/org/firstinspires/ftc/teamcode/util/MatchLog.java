package org.firstinspires.ftc.teamcode.util;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.Locale;

/**
 * A CSV "flight recorder" for every match: battery, loop time, flywheel, pose, faults...
 * Files go to /sdcard/FIRST/matchlogs on the robot (pull them off with Android Studio's
 * Device Explorer or adb), or ./matchlogs in the sim. Keeps the newest KEEP_FILES.
 *
 * It never throws: if anything goes wrong it just stops logging. A logger must not lose a match.
 */
public final class MatchLog {

    private static final int KEEP_FILES = 40;
    private static final double FLUSH_EVERY_SEC = 1.0;

    private BufferedWriter out;
    private final long startNanos = System.nanoTime();
    private long lastFlushNanos = startNanos;

    public MatchLog(String opModeName, String... columns) {
        try {
            File robotDir = new File("/sdcard/FIRST");
            File dir = robotDir.isDirectory() ? new File(robotDir, "matchlogs") : new File("matchlogs");
            if (!dir.isDirectory() && !dir.mkdirs()) return;
            prune(dir);
            String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            out = new BufferedWriter(new FileWriter(new File(dir, opModeName + "_" + stamp + ".csv")));
            StringBuilder header = new StringBuilder("t"); // no String.join on the hub's Android 7
            for (String column : columns) header.append(',').append(column);
            out.write(header.toString());
            out.newLine();
        } catch (Exception e) {
            out = null;
        }
    }

    public void row(Object... values) {
        if (out == null) return;
        try {
            StringBuilder sb = new StringBuilder(String.format(Locale.US, "%.3f", (System.nanoTime() - startNanos) / 1e9));
            for (Object v : values) {
                sb.append(',');
                if (v instanceof Double || v instanceof Float) sb.append(String.format(Locale.US, "%.3f", ((Number) v).doubleValue()));
                else sb.append(String.valueOf(v).replace(',', ';'));
            }
            out.write(sb.toString());
            out.newLine();
            long now = System.nanoTime();
            if ((now - lastFlushNanos) / 1e9 >= FLUSH_EVERY_SEC) {
                out.flush();
                lastFlushNanos = now;
            }
        } catch (Exception e) {
            out = null;
        }
    }

    public void close() {
        if (out == null) return;
        try {
            out.close();
        } catch (Exception ignored) {
            // nothing to do
        }
        out = null;
    }

    private static void prune(File dir) {
        File[] files = dir.listFiles((d, name) -> name.endsWith(".csv"));
        if (files == null || files.length < KEEP_FILES) return;
        Arrays.sort(files, (a, b) -> Long.compare(a.lastModified(), b.lastModified()));
        for (int i = 0; i <= files.length - KEEP_FILES; i++) {
            //noinspection ResultOfMethodCallIgnored
            files[i].delete();
        }
    }
}
