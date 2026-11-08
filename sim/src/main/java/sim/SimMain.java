package sim;

import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpServer;

import java.awt.Desktop;
import java.io.IOException;
import java.io.OutputStream;
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

/**
 * The sim with a web page on top: a driver station + 3D view at http://localhost:8080.
 *   .\gradlew -p sim run
 * All the actual sim lives in SimEngine.
 */
public final class SimMain {

    private final SimEngine engine = new SimEngine();
    private final Path webRoot = Paths.get("web").toAbsolutePath().normalize();

    public static void main(String[] args) throws Exception {
        int port = Integer.parseInt(System.getProperty("sim.port", System.getenv().getOrDefault("SIM_PORT", "8080")));
        new SimMain().run(port);
    }

    private void run(int port) throws Exception {
        engine.start();

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
        System.out.println("[sim] OpModes: " + engine.opModes().keySet());
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

    // ---------------------------------------------------------------- web

    private void handleEvents(HttpExchange ex) throws IOException {
        ex.getResponseHeaders().add("Content-Type", "text/event-stream");
        ex.getResponseHeaders().add("Cache-Control", "no-cache");
        ex.sendResponseHeaders(200, 0);
        try (OutputStream out = ex.getResponseBody()) {
            while (true) {
                out.write(("data: " + engine.stateJson() + "\n\n").getBytes(StandardCharsets.UTF_8));
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
        engine.setGamepad(v);
        reply(ex, 204, null, null);
    }

    private void handleCommand(HttpExchange ex) throws IOException {
        Map<String, String> q = query(ex.getRequestURI().getRawQuery());
        String action = q.getOrDefault("action", "");
        try {
            switch (action) {
                case "init":
                    engine.initOpMode(q.getOrDefault("opmode", ""));
                    break;
                case "start":
                    engine.startOpMode();
                    break;
                case "stop":
                    engine.stopOpMode();
                    break;
                case "reset":
                    engine.world().physics.resetRobot();
                    break;
                default:
                    reply(ex, 400, "text/plain", "unknown action".getBytes(StandardCharsets.UTF_8));
                    return;
            }
            reply(ex, 204, null, null);
        } catch (Throwable t) {
            reply(ex, 500, "text/plain", SimEngine.stackTrace(t).getBytes(StandardCharsets.UTF_8));
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
}
