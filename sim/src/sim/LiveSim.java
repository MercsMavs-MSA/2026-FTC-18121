package sim;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpServer;

import org.firstinspires.ftc.Maverick.Robot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.net.InetSocketAddress;
import java.net.URI;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * Live mode: runs Robot.java in real time (50 loops/s) and serves a page at http://localhost:8765
 * that reads your controller in Chrome and shows the field, telemetry and mechanism readouts.
 *
 * Phases work like the Driver Station: INIT (init() has run, loop() has not), RUNNING (after Start),
 * STOPPED (after Stop; press Init to set up again).
 */
public class LiveSim {

    static final int PORT = 8765;

    // Set up in the page; applied when the robot is (re)initialized.
    static volatile double startX = -30, startY = 0, startHeadingDeg = 0;
    static volatile int startBalls = 3;
    static volatile boolean fieldBalls = false;   // off by default so shot counts are easy to follow

    // Latest controller state from the page (HTTP thread writes, sim thread reads).
    static volatile float[] axes = new float[6];      // lx, ly, rx, ry, lt, rt
    static volatile int buttons = 0;                  // bit per button, see BTN_*
    static volatile long lastInputMs = 0;

    static final int BTN_A = 0, BTN_B = 1, BTN_X = 2, BTN_Y = 3, BTN_LB = 4, BTN_RB = 5, BTN_BACK = 8, BTN_START = 9,
            BTN_LSB = 10, BTN_RSB = 11, BTN_UP = 12, BTN_DOWN = 13, BTN_LEFT = 14, BTN_RIGHT = 15;

    static final ConcurrentLinkedQueue<String> commands = new ConcurrentLinkedQueue<>();
    static volatile String stateJson = "{}";

    // Owned by the sim thread
    static SimRunner.Harness h;
    static String phase = "INIT";
    static String error = "";
    static int runCount = 0;

    public static void main(String[] args) throws Exception {
        File page = new File("web/live.html");
        if (!page.exists()) {
            System.out.println("Cannot find web/live.html. Run this from the sim folder (run_sim.bat live does that).");
            return;
        }
        SimRunner.outDir = new File("out");
        initRobot();

        HttpServer server = HttpServer.create(new InetSocketAddress("127.0.0.1", PORT), 0);
        server.createContext("/", ex -> {
            if (!ex.getRequestURI().getPath().equals("/")) { send(ex, 404, "text/plain", "not found"); return; }
            send(ex, 200, "text/html; charset=utf-8", new String(Files.readAllBytes(page.toPath()), StandardCharsets.UTF_8));
        });
        server.createContext("/io", LiveSim::handleIo);
        server.setExecutor(Executors.newFixedThreadPool(2));
        server.start();

        ScheduledExecutorService loop = Executors.newSingleThreadScheduledExecutor();
        loop.scheduleAtFixedRate(LiveSim::tick, 0, (long) (SimWorld.DT * 1000), TimeUnit.MILLISECONDS);

        String url = "http://localhost:" + PORT;
        System.out.println("Live sim running. Open " + url + " in Chrome, plug in a controller and press any button on it.");
        System.out.println("Close this window (or Ctrl+C) to quit. Each run is logged to out/live_<n>.csv");
        try {
            if (java.awt.Desktop.isDesktopSupported()) java.awt.Desktop.getDesktop().browse(new URI(url));
        } catch (Throwable ignored) {
            // no desktop browser available: the URL is printed above
        }
    }

    // ---------------------------------------------------------------------------------------------
    // HTTP: the page posts its controller state and commands, and gets the latest state back.
    // Body: "lx,ly,rx,ry,lt,rt,buttons" then optional lines "cmd:..." .
    // ---------------------------------------------------------------------------------------------
    static void handleIo(HttpExchange ex) {
        try {
            String body = readBody(ex.getRequestBody());
            String[] lines = body.split("\n");
            if (lines.length > 0 && !lines[0].trim().isEmpty()) {
                String[] p = lines[0].trim().split(",");
                if (p.length >= 7) {
                    float[] a = new float[6];
                    for (int i = 0; i < 6; i++) a[i] = Float.parseFloat(p[i]);
                    axes = a;
                    buttons = Integer.parseInt(p[6]);
                    lastInputMs = System.currentTimeMillis();
                }
            }
            for (int i = 1; i < lines.length; i++) {
                String l = lines[i].trim();
                if (l.startsWith("cmd:")) commands.add(l.substring(4));
            }
            send(ex, 200, "application/json", stateJson);
        } catch (Exception e) {
            try { send(ex, 400, "text/plain", String.valueOf(e)); } catch (Exception ignored) {}
        }
    }

    static String readBody(InputStream in) throws Exception {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        byte[] buf = new byte[1024];
        int n;
        while ((n = in.read(buf)) > 0) out.write(buf, 0, n);
        return new String(out.toByteArray(), StandardCharsets.UTF_8);
    }

    static void send(HttpExchange ex, int code, String type, String body) throws java.io.IOException {
        byte[] b = body.getBytes(StandardCharsets.UTF_8);
        ex.getResponseHeaders().set("Content-Type", type);
        ex.getResponseHeaders().set("Cache-Control", "no-store");
        ex.sendResponseHeaders(code, b.length);
        try (OutputStream os = ex.getResponseBody()) { os.write(b); }
    }

    // ---------------------------------------------------------------------------------------------
    // Sim thread
    // ---------------------------------------------------------------------------------------------
    static void initRobot() {
        if (h != null) { try { h.stop(); } catch (Throwable ignored) {} }
        runCount++;
        double tx = h != null ? h.w.targetX : 66, ty = h != null ? h.w.targetY : 0;
        h = new SimRunner.Harness("live_" + runCount, startX, startY, startHeadingDeg, startBalls, tx, ty);
        h.w.fieldBallsAvailable = fieldBalls;
        error = "";
        try {
            h.robot.init();
            phase = "INIT";
        } catch (Throwable e) {
            fail(e);
        }
    }

    static void tick() {
        try {
            String c;
            while ((c = commands.poll()) != null) handleCommand(c);

            if (phase.equals("RUNNING")) {
                applyGamepad(h.g1);
                try {
                    h.stepOnce();
                } catch (Throwable e) {
                    fail(e);
                }
            } else if (phase.equals("INIT")) {
                h.w.step();   // physics keeps running (nothing should move); robot.loop() is not called
            }
            stateJson = buildState();
        } catch (Throwable e) {
            e.printStackTrace();
        }
    }

    static void fail(Throwable e) {
        StringWriter sw = new StringWriter();
        e.printStackTrace(new PrintWriter(sw));
        String[] lines = sw.toString().split("\n");
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < Math.min(6, lines.length); i++) sb.append(lines[i].trim()).append(" | ");
        error = "Robot code threw: " + sb;
        phase = "STOPPED";
        System.out.println(sw);
    }

    static void handleCommand(String c) {
        String[] p = c.split(" ");
        switch (p[0]) {
            case "init":
                initRobot();
                break;
            case "start":
                if (phase.equals("INIT")) {
                    try { h.start(); phase = "RUNNING"; } catch (Throwable e) { fail(e); }
                }
                break;
            case "stop":
                if (phase.equals("RUNNING")) {
                    try { h.robot.stop(); } catch (Throwable e) { fail(e); }
                    phase = "STOPPED";
                }
                break;
            case "target":   // target x y : any time
                double tx = Double.parseDouble(p[1]), ty = Double.parseDouble(p[2]);
                Robot.TARGET_X = tx;
                Robot.TARGET_Y = ty;
                h.w.targetX = tx;
                h.w.targetY = ty;
                break;
            case "place":    // place x y headingDeg : sets the start pose, then re-inits
                startX = Double.parseDouble(p[1]);
                startY = Double.parseDouble(p[2]);
                startHeadingDeg = Double.parseDouble(p[3]);
                initRobot();
                break;
            case "balls":    // balls n : start loaded with n
                startBalls = Integer.parseInt(p[1]);
                h.w.balls = Math.min(SimWorld.MAX_BALLS, startBalls);
                break;
            case "addball":
                if (h.w.balls < SimWorld.MAX_BALLS) h.w.balls++;
                break;
            case "fieldballs":
                fieldBalls = p[1].equals("1");
                h.w.fieldBallsAvailable = fieldBalls;
                break;
            default:
                break;
        }
    }

    static void applyGamepad(Gamepad g) {
        g.clear();
        boolean fresh = System.currentTimeMillis() - lastInputMs < 500;   // page closed or tab hidden: let go of everything
        if (!fresh) return;
        float[] a = axes;
        int b = buttons;
        g.left_stick_x = a[0];
        g.left_stick_y = a[1];
        g.right_stick_x = a[2];
        g.right_stick_y = a[3];
        g.left_trigger = a[4];
        g.right_trigger = a[5];
        g.a = bit(b, BTN_A); g.b = bit(b, BTN_B); g.x = bit(b, BTN_X); g.y = bit(b, BTN_Y);
        g.left_bumper = bit(b, BTN_LB); g.right_bumper = bit(b, BTN_RB);
        g.back = bit(b, BTN_BACK); g.start = bit(b, BTN_START);
        g.left_stick_button = bit(b, BTN_LSB); g.right_stick_button = bit(b, BTN_RSB);
        g.dpad_up = bit(b, BTN_UP); g.dpad_down = bit(b, BTN_DOWN); g.dpad_left = bit(b, BTN_LEFT); g.dpad_right = bit(b, BTN_RIGHT);
    }

    static boolean bit(int b, int i) { return (b & (1 << i)) != 0; }

    // ---------------------------------------------------------------------------------------------
    static String buildState() {
        SimWorld w = h.w;
        Pose2D est = w.pinpoint.getPosition();
        StringBuilder sb = new StringBuilder(2048);
        sb.append('{');
        kv(sb, "phase", phase); sb.append(',');
        kv(sb, "error", error); sb.append(',');
        num(sb, "t", h.t); sb.append(',');
        num(sb, "x", w.x); sb.append(','); num(sb, "y", w.y); sb.append(','); num(sb, "h", Math.toDegrees(w.heading)); sb.append(',');
        num(sb, "ex", est.getX(DistanceUnit.INCH)); sb.append(','); num(sb, "ey", est.getY(DistanceUnit.INCH)); sb.append(',');
        num(sb, "eh", est.getHeading(AngleUnit.DEGREES)); sb.append(',');
        num(sb, "turret", w.turretDeg()); sb.append(',');
        num(sb, "turretTarget", h.turretTargetDeg()); sb.append(',');
        num(sb, "rpm", w.wheelRpm()); sb.append(','); num(sb, "rpmTarget", w.flywheelTargetRpm()); sb.append(',');
        bool(sb, "gateOpen", w.gatePhysicallyOpen()); sb.append(','); bool(sb, "gateCmd", w.gateCommandedOpen()); sb.append(',');
        sb.append("\"balls\":").append(w.balls).append(',');
        bool(sb, "fieldBalls", w.fieldBallsAvailable); sb.append(',');
        num(sb, "flower", Double.isNaN(w.flower.commanded) ? -1 : w.flower.commanded); sb.append(',');
        num(sb, "intake", w.intake.power); sb.append(','); num(sb, "transfer", w.transfer.power); sb.append(',');
        num(sb, "tx", w.targetX); sb.append(','); num(sb, "ty", w.targetY); sb.append(',');
        num(sb, "turretOffF", org.firstinspires.ftc.Maverick.Subsystems.Shooter.ShooterCalculator.TURRET_OFFSET_FORWARD_IN); sb.append(',');
        num(sb, "turretOffL", org.firstinspires.ftc.Maverick.Subsystems.Shooter.ShooterCalculator.TURRET_OFFSET_LEFT_IN); sb.append(',');
        String vr = h.tel.get("Shot valid / ready");
        bool(sb, "valid", vr.startsWith("true")); sb.append(','); bool(sb, "ready", vr.endsWith("true")); sb.append(',');
        bool(sb, "hardStop", w.turretHitHardStop); sb.append(','); bool(sb, "braked", w.flywheelBraked); sb.append(',');
        sb.append("\"shotCount\":").append(w.shots.size()).append(',');
        sb.append("\"shots\":[");
        int from = Math.max(0, w.shots.size() - 8);
        for (int i = from; i < w.shots.size(); i++) {
            SimWorld.Shot s = w.shots.get(i);
            if (i > from) sb.append(',');
            sb.append(String.format(Locale.US, "{\"n\":%d,\"t\":%.2f,\"rpm\":%.0f,\"target\":%.0f,\"rpmErr\":%.1f,\"aimErr\":%.1f,\"tol\":%.1f,\"dist\":%.0f,\"rpmOk\":%b,\"aimOk\":%b}",
                    i + 1, s.t - h.startWorldT, s.wheelRpm, s.targetRpm, s.rpmErrPct, s.aimErrDeg, s.aimTolDeg, s.distanceIn, s.rpmOk(), s.aimOk()));
        }
        sb.append("],");
        sb.append("\"telemetry\":[");
        boolean first = true;
        for (Map.Entry<String, String> e : h.tel.lastFrame.entrySet()) {
            if (!first) sb.append(',');
            first = false;
            sb.append('[').append(q(e.getKey().startsWith("line") ? "" : e.getKey())).append(',').append(q(e.getValue())).append(']');
        }
        sb.append("]}");
        return sb.toString();
    }

    static void kv(StringBuilder sb, String k, String v) { sb.append(q(k)).append(':').append(q(v)); }
    static void num(StringBuilder sb, String k, double v) {
        sb.append(q(k)).append(':').append(Double.isNaN(v) || Double.isInfinite(v) ? "null" : String.format(Locale.US, "%.3f", v));
    }
    static void bool(StringBuilder sb, String k, boolean v) { sb.append(q(k)).append(':').append(v); }

    static String q(String s) {
        StringBuilder sb = new StringBuilder("\"");
        for (char c : s.toCharArray()) {
            if (c == '"' || c == '\\') sb.append('\\').append(c);
            else if (c < 0x20) sb.append(' ');
            else sb.append(c);
        }
        return sb.append('"').toString();
    }
}
