package sim;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.SimOpModeRunner;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.io.File;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.lang.reflect.Modifier;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Stream;

/**
 * The sim without the web page: fake robot world, physics loop, and the OpMode lifecycle
 * (INIT / START / STOP) the way the Robot Controller app does it. SimMain puts a web server on
 * top of this; the tests drive it directly.
 */
public final class SimEngine {

    private static final int PHYSICS_HZ = 200;

    private final SimWorld world = new SimWorld();
    private final SimTelemetry telemetry = new SimTelemetry();
    private final Gamepad input1 = new Gamepad();
    private final Gamepad input2 = new Gamepad();
    private final Map<String, Class<? extends OpMode>> opModes = new LinkedHashMap<>();
    private final Map<String, Boolean> opModeIsAuto = new LinkedHashMap<>();

    private volatile SimOpModeRunner runner;
    private volatile String opModeName = null;
    private volatile String errorText = null;
    private Thread physicsThread;

    public SimEngine() {
        findOpModes();
    }

    public SimWorld world() {
        return world;
    }

    public Map<String, Boolean> opModes() {
        return opModeIsAuto;
    }

    public String opModeName() {
        return opModeName;
    }

    public String error() {
        return errorText;
    }

    public List<String> telemetryLines() {
        return telemetry.lines();
    }

    public void start() {
        physicsThread = new Thread(this::physicsLoop, "sim-physics");
        physicsThread.setDaemon(true);
        physicsThread.start();
    }

    public void shutdown() {
        stopOpMode();
        if (physicsThread != null) physicsThread.interrupt();
    }

    // ---------------------------------------------------------------- OpModes

    @SuppressWarnings("unchecked")
    private void findOpModes() {
        try {
            Path classes = Paths.get(OpModeLocator.class.getProtectionDomain().getCodeSource().getLocation().toURI());
            Path teamCode = classes.resolve("org/firstinspires/ftc/teamcode");
            List<String> names = new ArrayList<>();
            try (Stream<Path> files = Files.walk(teamCode)) {
                files.filter(p -> p.toString().endsWith(".class") && !p.getFileName().toString().contains("$"))
                        .forEach(p -> names.add(classes.relativize(p).toString()
                                .replace(File.separatorChar, '.').replaceAll("\\.class$", "")));
            }
            names.sort(String::compareTo);
            for (String className : names) {
                try {
                    Class<?> c = Class.forName(className, false, SimEngine.class.getClassLoader());
                    if (!OpMode.class.isAssignableFrom(c) || Modifier.isAbstract(c.getModifiers())) continue;
                    if (c.isAnnotationPresent(Disabled.class)) continue;
                    TeleOp teleOp = c.getAnnotation(TeleOp.class);
                    Autonomous auto = c.getAnnotation(Autonomous.class);
                    if (teleOp == null && auto == null) continue;
                    String name = teleOp != null ? teleOp.name() : auto.name();
                    if (name.isEmpty()) name = c.getSimpleName();
                    opModes.put(name, (Class<? extends OpMode>) c);
                    opModeIsAuto.put(name, auto != null);
                } catch (Throwable t) {
                    System.out.println("[sim] skipped " + className + ": " + t);
                }
            }
        } catch (Exception e) {
            throw new IllegalStateException("couldn't find the OpModes", e);
        }
    }

    // anything from the main classes dir works as an anchor
    private static final class OpModeLocator {}

    public synchronized void initOpMode(String name) throws Exception {
        stopOpMode();
        Class<? extends OpMode> c = opModes.get(name);
        if (c == null) throw new IllegalArgumentException("No OpMode named " + name);
        boolean auto = opModeIsAuto.get(name);
        errorText = null;
        world.resetForNewOpMode(auto);
        world.phase = SimWorld.Phase.INIT;
        opModeName = name;
        try {
            OpMode opMode = c.getDeclaredConstructor().newInstance();
            runner = new SimOpModeRunner(opMode, new SimHardwareMap(world), telemetry.telemetry, input1, input2);
            runner.init();
        } catch (Throwable t) {
            world.phase = SimWorld.Phase.STOPPED;
            errorText = stackTrace(t);
            throw t;
        }
    }

    public synchronized void startOpMode() {
        if (runner == null || world.phase != SimWorld.Phase.INIT) return;
        runner.start();
        world.phase = SimWorld.Phase.RUNNING;
    }

    public synchronized void stopOpMode() {
        if (runner == null) return;
        runner.stop();
        if (runner.getError() != null && errorText == null) errorText = stackTrace(runner.getError());
        runner = null;
        world.phase = SimWorld.Phase.STOPPED;
        disableOutputs();
    }

    // when an OpMode ends the SDK cuts the motors and the servos go limp
    private void disableOutputs() {
        for (SimMotor m : world.motors()) m.power = 0;
        for (SimServo s : world.servos()) s.commanded = Double.NaN;
    }

    // ---------------------------------------------------------------- gamepad

    /**
     * Sets gamepad 1 from short keys: lx ly rx ry lt rt (numbers), a b x y lb rb up down left
     * right start back ls rs (booleans). Anything not given is released.
     */
    public void setGamepad(Map<String, ?> v) {
        synchronized (input1) {
            Gamepad g = input1;
            g.left_stick_x = f(v, "lx");
            g.left_stick_y = f(v, "ly");
            g.right_stick_x = f(v, "rx");
            g.right_stick_y = f(v, "ry");
            g.left_trigger = f(v, "lt");
            g.right_trigger = f(v, "rt");
            g.a = b(v, "a");
            g.b = b(v, "b");
            g.x = b(v, "x");
            g.y = b(v, "y");
            g.left_bumper = b(v, "lb");
            g.right_bumper = b(v, "rb");
            g.dpad_up = b(v, "up");
            g.dpad_down = b(v, "down");
            g.dpad_left = b(v, "left");
            g.dpad_right = b(v, "right");
            g.start = b(v, "start");
            g.back = b(v, "back");
            g.left_stick_button = b(v, "ls");
            g.right_stick_button = b(v, "rs");
        }
    }

    private static float f(Map<String, ?> v, String key) {
        Object value = v.get(key);
        if (value instanceof Number) return ((Number) value).floatValue();
        try {
            return value == null ? 0f : Float.parseFloat(value.toString());
        } catch (NumberFormatException e) {
            return 0f;
        }
    }

    private static boolean b(Map<String, ?> v, String key) {
        Object value = v.get(key);
        return Boolean.TRUE.equals(value) || "true".equals(value);
    }

    // ---------------------------------------------------------------- physics loop

    private void physicsLoop() {
        long periodNanos = 1_000_000_000L / PHYSICS_HZ;
        long last = System.nanoTime();
        int count = 0;
        while (!Thread.currentThread().isInterrupted()) {
            long now = System.nanoTime();
            double dt = Math.min((now - last) / 1e9, 0.05);
            last = now;
            try {
                world.physics.step(dt);
                SimOpModeRunner r = runner;
                if (r != null && count++ % 2 == 0) {
                    r.tick();
                    if (r.isFinished()) {
                        synchronized (this) {
                            if (runner == r) {
                                if (r.getError() != null) errorText = stackTrace(r.getError());
                                runner = null;
                                world.phase = SimWorld.Phase.STOPPED;
                                disableOutputs();
                            }
                        }
                    }
                }
            } catch (Throwable t) {
                System.out.println("[sim] physics error: " + t);
            }
            long sleepNanos = periodNanos - (System.nanoTime() - now);
            if (sleepNanos > 0) {
                try {
                    Thread.sleep(sleepNanos / 1_000_000L, (int) (sleepNanos % 1_000_000L));
                } catch (InterruptedException e) {
                    return;
                }
            }
        }
    }

    // ---------------------------------------------------------------- state for the browser

    public String stateJson() {
        SimPhysics p = world.physics;
        double[] otos = world.otos().reportedPose();
        Json j = new Json().begin();
        j.field("phase", world.phase.name());
        j.field("opmode", opModeName);
        j.field("autonomous", world.autonomous);
        j.field("error", errorText);
        j.key("robot").begin()
                .field("x", p.x).field("y", p.y).field("h", p.heading)
                .field("vForward", p.vForward).field("vLeft", p.vLeft).field("vTurn", p.vTurn)
                .end();
        j.key("otos").begin().field("x", otos[0]).field("y", otos[1]).field("h", otos[2]).end();
        j.field("battery", p.batteryVolts);
        j.field("flywheelRpm", p.flywheelRpm);
        j.field("intakeRpm", p.intakeRpm);
        j.field("turretDeg", p.turretDeg);
        j.field("balls", p.balls);
        j.field("shots", p.shotsLaunched);
        j.field("jammed", p.jammed);
        j.key("servos").begin();
        for (SimServo s : world.servos()) {
            j.key(s.name).begin().field("cmd", s.commanded).field("pos", s.physical).end();
        }
        j.end();
        j.key("motors").begin();
        for (SimMotor m : world.motors()) {
            j.key(m.name).begin().field("power", m.power).field("rpm", m.mechanismRpm).end();
        }
        j.end();
        j.stringArray("telemetry", telemetry.lines());
        j.stringArray("warnings", world.warnings());
        j.stringArray("devices", world.requestedDevices());
        j.key("opmodes").beginArray();
        for (Map.Entry<String, Boolean> e : opModeIsAuto.entrySet()) {
            j.begin().field("name", e.getKey()).field("auto", e.getValue()).end();
        }
        j.endArray();
        return j.end().toString();
    }

    static String stackTrace(Throwable t) {
        StringWriter sw = new StringWriter();
        t.printStackTrace(new PrintWriter(sw));
        return sw.toString();
    }
}
