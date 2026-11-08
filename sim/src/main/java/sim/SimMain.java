package sim;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.SimOpModeRunner;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpServer;

import java.awt.Desktop;
import java.io.File;
import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.lang.reflect.Modifier;
import java.net.InetSocketAddress;
import java.net.URI;
import java.net.URLDecoder;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Stream;

/**
 * Runs the sim: physics loop, the OpMode, and a little web server for the 3D view.
 *   .\gradlew -p sim run      then open http://localhost:8080
 */
public final class SimMain {

    private static final int PHYSICS_HZ = 200;

    private final SimWorld world = new SimWorld();
    private final SimTelemetry telemetry = new SimTelemetry();
    private final Gamepad input1 = new Gamepad();
    private final Gamepad input2 = new Gamepad();
    private final Map<String, Class<? extends OpMode>> opModes = new LinkedHashMap<>();
    private final Map<String, Boolean> opModeIsAuto = new LinkedHashMap<>();
    private final Path webRoot = Paths.get("web").toAbsolutePath().normalize();

    private volatile SimOpModeRunner runner;
    private volatile String opModeName = null;
    private volatile String errorText = null;

    public static void main(String[] args) throws Exception {
        int port = Integer.parseInt(System.getProperty("sim.port", System.getenv().getOrDefault("SIM_PORT", "8080")));
        new SimMain().run(port);
    }

    private void run(int port) throws Exception {
        findOpModes();

        Thread physicsThread = new Thread(this::physicsLoop, "sim-physics");
        physicsThread.setDaemon(true);
        physicsThread.start();

        // 0.0.0.0 = reachable from other computers on the network, SIM_HOST=localhost keeps it to this PC
        String host = System.getProperty("sim.host", System.getenv().getOrDefault("SIM_HOST", "0.0.0.0"));
        HttpServer server = HttpServer.create(new InetSocketAddress(host, port), 0);
        server.setExecutor(Executors.newCachedThreadPool());
        server.createContext("/events", this::handleEvents);
        server.createContext("/gamepad", this::handleGamepad);
        server.createContext("/cmd", this::handleCommand);
        server.createContext("/", this::handleStatic);
        server.start();

        String url = "http://localhost:" + port;
        System.out.println("[sim] OpModes: " + opModes.keySet());
        System.out.println("[sim] Running at " + url + "  (Ctrl+C to quit)");
        if (!host.equals("localhost") && !host.equals("127.0.0.1")) {
            for (String lanUrl : lanUrls(port)) System.out.println("[sim] On your network: " + lanUrl);
        }
        if (!Boolean.getBoolean("sim.noBrowser") && System.getenv("SIM_NO_BROWSER") == null) {
            try {
                if (Desktop.isDesktopSupported()) Desktop.getDesktop().browse(new URI(url));
            } catch (Exception ignored) {
                // open it yourself
            }
        }
    }

    // addresses other computers can reach this one at (skips loopback, VPNs and virtual adapters)
    private static List<String> lanUrls(int port) {
        List<String> urls = new ArrayList<>();
        try {
            for (java.net.NetworkInterface nic : java.util.Collections.list(java.net.NetworkInterface.getNetworkInterfaces())) {
                if (!nic.isUp() || nic.isLoopback() || nic.isVirtual()) continue;
                String label = nic.getDisplayName().toLowerCase(java.util.Locale.ROOT);
                if (label.contains("virtual") || label.contains("vethernet") || label.contains("hyper-v")
                        || label.contains("vmware") || label.contains("wsl")) continue;
                for (java.net.InetAddress address : java.util.Collections.list(nic.getInetAddresses())) {
                    if (address instanceof java.net.Inet4Address && address.isSiteLocalAddress()) {
                        urls.add("http://" + address.getHostAddress() + ":" + port);
                    }
                }
            }
        } catch (Exception ignored) {
            // no network info, localhost still works
        }
        return urls;
    }

    // ---------------------------------------------------------------- OpModes

    @SuppressWarnings("unchecked")
    private void findOpModes() throws Exception {
        Path classes;
        try {
            classes = Paths.get(SimMain.class.getProtectionDomain().getCodeSource().getLocation().toURI());
        } catch (Exception e) {
            throw new IllegalStateException(e);
        }
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
                Class<?> c = Class.forName(className, false, SimMain.class.getClassLoader());
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
    }

    private synchronized void initOpMode(String name) throws Exception {
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

    private synchronized void startOpMode() {
        if (runner == null || world.phase != SimWorld.Phase.INIT) return;
        runner.start();
        world.phase = SimWorld.Phase.RUNNING;
    }

    private synchronized void stopOpMode() {
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

    // ---------------------------------------------------------------- physics loop

    private void physicsLoop() {
        long periodNanos = 1_000_000_000L / PHYSICS_HZ;
        long last = System.nanoTime();
        int count = 0;
        while (true) {
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

    // ---------------------------------------------------------------- web

    private void handleEvents(HttpExchange ex) throws IOException {
        ex.getResponseHeaders().add("Content-Type", "text/event-stream");
        ex.getResponseHeaders().add("Cache-Control", "no-cache");
        ex.sendResponseHeaders(200, 0);
        try (OutputStream out = ex.getResponseBody()) {
            while (true) {
                out.write(("data: " + stateJson() + "\n\n").getBytes(StandardCharsets.UTF_8));
                out.flush();
                Thread.sleep(33);
            }
        } catch (IOException | InterruptedException ignored) {
            // browser went away
        }
    }

    private static final Pattern JSON_FIELD = Pattern.compile("\"(\\w+)\"\\s*:\\s*(true|false|-?[0-9.eE+-]+)");

    private void handleGamepad(HttpExchange ex) throws IOException {
        String body = new String(ex.getRequestBody().readAllBytes(), StandardCharsets.UTF_8);
        Map<String, String> v = new LinkedHashMap<>();
        Matcher m = JSON_FIELD.matcher(body);
        while (m.find()) v.put(m.group(1), m.group(2));
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
        reply(ex, 204, null, null);
    }

    private static float f(Map<String, String> v, String key) {
        try {
            return v.containsKey(key) ? Float.parseFloat(v.get(key)) : 0f;
        } catch (NumberFormatException e) {
            return 0f;
        }
    }

    private static boolean b(Map<String, String> v, String key) {
        return "true".equals(v.get(key));
    }

    private void handleCommand(HttpExchange ex) throws IOException {
        Map<String, String> q = query(ex.getRequestURI().getRawQuery());
        String action = q.getOrDefault("action", "");
        try {
            switch (action) {
                case "init":
                    initOpMode(q.getOrDefault("opmode", ""));
                    break;
                case "start":
                    startOpMode();
                    break;
                case "stop":
                    stopOpMode();
                    break;
                case "reset":
                    world.physics.resetRobot();
                    break;
                default:
                    reply(ex, 400, "text/plain", "unknown action".getBytes(StandardCharsets.UTF_8));
                    return;
            }
            reply(ex, 204, null, null);
        } catch (Throwable t) {
            reply(ex, 500, "text/plain", stackTrace(t).getBytes(StandardCharsets.UTF_8));
        }
    }

    private void handleStatic(HttpExchange ex) throws IOException {
        String path = URLDecoder.decode(ex.getRequestURI().getPath(), StandardCharsets.UTF_8);
        if (path.equals("/")) path = "/index.html";
        Path file = webRoot.resolve(path.substring(1)).normalize();
        if (!file.startsWith(webRoot) || !Files.isRegularFile(file)) {
            reply(ex, 404, "text/plain", "not found".getBytes(StandardCharsets.UTF_8));
            return;
        }
        reply(ex, 200, contentType(file.toString()), Files.readAllBytes(file));
    }

    private static String contentType(String name) {
        if (name.endsWith(".html")) return "text/html; charset=utf-8";
        if (name.endsWith(".js")) return "text/javascript; charset=utf-8";
        if (name.endsWith(".json")) return "application/json";
        if (name.endsWith(".css")) return "text/css";
        if (name.endsWith(".glb")) return "model/gltf-binary";
        if (name.endsWith(".gltf")) return "model/gltf+json";
        if (name.endsWith(".png")) return "image/png";
        return "application/octet-stream";
    }

    private static void reply(HttpExchange ex, int code, String type, byte[] body) throws IOException {
        if (type != null) ex.getResponseHeaders().add("Content-Type", type);
        ex.getResponseHeaders().add("Cache-Control", "no-store");
        if (body == null) {
            ex.sendResponseHeaders(code, -1);
        } else {
            ex.sendResponseHeaders(code, body.length);
            try (OutputStream out = ex.getResponseBody()) {
                out.write(body);
            }
        }
        ex.close();
    }

    private static Map<String, String> query(String raw) {
        Map<String, String> map = new LinkedHashMap<>();
        if (raw == null) return map;
        for (String pair : raw.split("&")) {
            int i = pair.indexOf('=');
            if (i < 0) continue;
            map.put(URLDecoder.decode(pair.substring(0, i), StandardCharsets.UTF_8),
                    URLDecoder.decode(pair.substring(i + 1), StandardCharsets.UTF_8));
        }
        return map;
    }

    // ---------------------------------------------------------------- state for the browser

    private String stateJson() {
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

    private static String stackTrace(Throwable t) {
        StringWriter sw = new StringWriter();
        t.printStackTrace(new PrintWriter(sw));
        return sw.toString();
    }
}
