package sim;

import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;

/**
 * Drives the sim from a test like a drive team would: INIT, START, hold buttons, wait, read the
 * Driver Hub telemetry. Runs in real time (the robot code uses the real clock).
 */
final class SimHarness implements AutoCloseable {

    final SimEngine engine = new SimEngine();
    final SimWorld world = engine.world();
    final SimPhysics physics = world.physics();
    private final Map<String, Object> pad = new HashMap<>();

    SimHarness() {
        engine.start();
    }

    SimHarness init(String opMode) throws Exception {
        engine.initOpMode(opMode);
        sleep(800); // init + a few init_loops
        assertNoCrash();
        return this;
    }

    SimHarness start() {
        engine.startOpMode();
        sleep(300);
        assertNoCrash();
        return this;
    }

    SimHarness stop() {
        engine.stopOpMode();
        assertNoCrash();
        return this;
    }

    SimHarness hold(String key, Object value) {
        pad.put(key, value);
        engine.setGamepad(pad);
        return this;
    }

    SimHarness release(String key) {
        pad.remove(key);
        engine.setGamepad(pad);
        return this;
    }

    SimHarness press(String key) {
        hold(key, true);
        sleep(150);
        return release(key);
    }

    /**
     * Waits until cond is true or the time runs out, returns whether it happened
     */
    boolean waitUntil(BooleanSupplier cond, double seconds) {
        long end = System.nanoTime() + (long) (seconds * 1e9);
        while (System.nanoTime() < end) {
            if (cond.getAsBoolean()) return true;
            sleep(20);
            assertNoCrash();
        }
        return cond.getAsBoolean();
    }

    /**
     * Every distinct value a telemetry line had while waiting (for statuses that only last a moment)
     */
    Set<String> watch(String caption, double seconds, BooleanSupplier stopEarly) {
        Set<String> seen = new LinkedHashSet<>();
        long end = System.nanoTime() + (long) (seconds * 1e9);
        while (System.nanoTime() < end) {
            String value = telemetry(caption);
            if (value != null) seen.add(value);
            if (stopEarly != null && stopEarly.getAsBoolean()) break;
            sleep(20);
            assertNoCrash();
        }
        return seen;
    }

    /**
     * The value of a "Caption : value" telemetry line, or null
     */
    String telemetry(String caption) {
        for (String line : engine.telemetryLines()) {
            if (line.startsWith(caption + " : ")) return line.substring(caption.length() + 3);
            if (line.equals(caption)) return "";
        }
        return null;
    }

    List<String> telemetryLines() {
        return engine.telemetryLines();
    }

    void assertNoCrash() {
        if (engine.error() != null) throw new AssertionError("OpMode crashed:\n" + engine.error());
    }

    static void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    @Override
    public void close() {
        engine.shutdown();
    }
}
