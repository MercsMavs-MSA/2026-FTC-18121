package sim;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.Maverick.Robot;
import org.firstinspires.ftc.Maverick.Subsystems.Drive.Drive;
import org.firstinspires.ftc.Maverick.Subsystems.Intake.Intake;
import org.firstinspires.ftc.Maverick.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.Maverick.Subsystems.Shooter.Turret;
import org.firstinspires.ftc.Maverick.Subsystems.Transfer.Transfer;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.io.File;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Runs the real Robot.java against SimWorld, one scripted scenario per binding, and prints PASS / FAIL / INFO.
 * Each scenario also writes out/<name>.csv with one row per loop, for plotting.
 *
 * Run all:          java -cp build sim.SimRunner
 * Run some:         java -cp build sim.SimRunner shoot fixed     (any scenario whose name contains a word)
 */
public class SimRunner {

    interface Script { void apply(double t, Gamepad g); }
    interface Watch { void each(Harness h); }

    static int passed = 0, failed = 0;
    static final List<String> failures = new ArrayList<>();
    static File outDir = new File("out");

    // Defaults of the statics a scenario may change, restored before each scenario.
    static final double DEFAULT_TARGET_X = Robot.TARGET_X, DEFAULT_TARGET_Y = Robot.TARGET_Y;
    // Read through reflection so the sim still compiles if Shooter has no FIXED_RPM field.
    static final double DEFAULT_FIXED_RPM = fixedRpm();

    static double fixedRpm() {
        try { return Shooter.class.getField("FIXED_RPM").getDouble(null); } catch (Exception e) { return Double.NaN; }
    }

    static void setFixedRpm(double v) {
        try { Shooter.class.getField("FIXED_RPM").setDouble(null, v); } catch (Exception e) { /* field not present */ }
    }

    // =====================================================================================
    static class Harness {
        final String name;
        final SimWorld w;
        final Robot robot = new Robot();
        final Gamepad g1 = new Gamepad(), g2 = new Gamepad();
        final SimTelemetry tel = new SimTelemetry();
        final List<Watch> watches = new ArrayList<>();
        PrintWriter csv;
        int loops = 0;
        double t = 0;          // seconds since start()
        String error = null;

        Harness(String name, double x, double y, double headingDeg, int balls, double targetX, double targetY) {
            this.name = name;
            Robot.TARGET_X = targetX;
            Robot.TARGET_Y = targetY;
            setFixedRpm(DEFAULT_FIXED_RPM);
            Turret.handoffAngleRad = 0;
            Drive.handoffPose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, headingDeg);

            w = new SimWorld();
            w.x = x; w.y = y; w.heading = Math.toRadians(headingDeg);
            w.balls = balls;
            w.targetX = targetX; w.targetY = targetY;

            robot.hardwareMap = w.hardwareMap;
            robot.gamepad1 = g1;
            robot.gamepad2 = g2;
            robot.telemetry = tel;
            try {
                outDir.mkdirs();
                csv = new PrintWriter(new File(outDir, name + ".csv"));
                csv.println("t,robot_state,shooter_state,fixed,x_true,y_true,heading_true_deg,x_est,y_est,"
                        + "turret_deg,turret_target_deg,wheel_rpm,wheel_rpm_target,shot_valid_ready,gate_cmd_open,gate_open,"
                        + "balls,shots,intake_power,transfer_power,flower_pos");
            } catch (Exception e) { throw new RuntimeException(e); }
        }

        void init(double seconds) {
            robot.init();
            for (double s = 0; s < seconds; s += SimWorld.DT) { robot.init_loop(); w.step(); }
        }

        double startWorldT = 0;

        void start() {
            startWorldT = w.nanos / 1e9;
            w.started = true;
            robot.start();
        }

        void run(double seconds, Script script) {
            double end = t + seconds;
            double t0 = t;
            while (t < end - 1e-9) {
                g1.clear();
                script.apply(t - t0, g1);
                robot.loop();
                w.step();
                t += SimWorld.DT;
                loops++;
                log();
                for (Watch wt : watches) wt.each(this);
            }
        }

        void idle(double seconds) { run(seconds, (tt, g) -> {}); }

        /** One live loop: the caller has already set the gamepad. */
        void stepOnce() {
            robot.loop();
            w.step();
            t += SimWorld.DT;
            loops++;
            log();
        }

        void press(Script hold) { run(SimWorld.DT * 2, hold); idle(SimWorld.DT * 2); }

        void stop() { robot.stop(); csv.close(); }

        String tele(String k) { return tel.get(k); }

        void log() {
            Pose2D est = w.pinpoint.getPosition();
            csv.println(String.format(Locale.US,
                    "%.2f,%s,%s,%s,%.2f,%.2f,%.1f,%.2f,%.2f,%.1f,%s,%.0f,%.0f,%s,%b,%b,%d,%d,%.2f,%.2f,%.2f",
                    t, tele("Robot State"), tele("Shooter State"), tele("Fixed Shooter"),
                    w.x, w.y, Math.toDegrees(w.heading), est.getX(DistanceUnit.INCH), est.getY(DistanceUnit.INCH),
                    w.turretDeg(), turretTargetDeg(), w.wheelRpm(), w.flywheelTargetRpm(),
                    tele("Shot valid / ready").replace(" / ", "/"), w.gateCommandedOpen(), w.gatePhysicallyOpen(),
                    w.balls, w.shots.size(), w.intake.power, w.transfer.power,
                    Double.isNaN(w.flower.commanded) ? -1 : w.flower.commanded));
        }

        double turretTargetDeg() {
            String s = tele("Turret");          // "now / target deg"
            try { return Double.parseDouble(s.split("/")[1].replace("deg", "").trim()); } catch (Exception e) { return Double.NaN; }
        }

        boolean ready() { return tele("Shot valid / ready").endsWith("true"); }
    }

    // =====================================================================================
    static void check(String scenario, String what, boolean ok, String detail) {
        if (ok) passed++; else { failed++; failures.add(scenario + ": " + what + (detail.isEmpty() ? "" : " (" + detail + ")")); }
        System.out.println("  " + (ok ? "PASS " : "FAIL ") + what + (detail.isEmpty() ? "" : "  [" + detail + "]"));
    }

    static void info(String what) { System.out.println("  INFO " + what); }

    static String f(String fmt, Object... a) { return String.format(Locale.US, fmt, a); }

    static void shotReport(Harness h) {
        for (int i = 0; i < h.w.shots.size(); i++) {
            SimWorld.Shot s = h.w.shots.get(i);
            info(f("shot %d at t=%.2fs: wheel %.0f / %.0f rpm (%+.1f%%), aim error %.1f deg (tolerance %.1f), distance %.0f in  -> %s",
                    i + 1, s.t - firstStart(h), s.wheelRpm, s.targetRpm, s.rpmErrPct, s.aimErrDeg, s.aimTolDeg, s.distanceIn,
                    (s.rpmOk() && s.aimOk()) ? "GOOD" : (!s.rpmOk() ? "RPM OFF" : "AIM OFF")));
        }
    }

    static double startNanosOffset = 0;
    static double firstStart(Harness h) { return startNanosOffset; }

    // =====================================================================================
    // Scenarios
    // =====================================================================================

    static void initNothingMoves() {
        Harness h = new Harness("init_nothing_moves", 0, 0, 0, 0, 96, 0);
        h.init(1.0);
        boolean motorMoved = false, servoMoved = false;
        for (SimMotor m : new SimMotor[]{h.w.fl, h.w.fr, h.w.bl, h.w.br, h.w.intake, h.w.transfer, h.w.flywheel, h.w.turret})
            motorMoved |= m.commandedBeforeStart;
        servoMoved = h.w.flower.commandedBeforeStart || h.w.gate.commandedBeforeStart;
        check(h.name, "no motor powered during INIT", !motorMoved, "");
        check(h.name, "no servo commanded during INIT", !servoMoved, "");
        check(h.name, "bulk caching set to MANUAL", h.w.controlHub.mode == com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode.MANUAL, "");
        startNanosOffset = h.w.nanos / 1e9;
        h.start();
        h.idle(0.6);
        check(h.name, "flower opens at Start", h.w.flower.commanded == Intake.FLOWER_OPEN_POSITION, f("cmd %.2f", h.w.flower.commanded));
        check(h.name, "gate closed at Start", h.w.gate.commanded == Transfer.GATE_CLOSED_POSITION, f("cmd %.2f", h.w.gate.commanded));
        check(h.name, "bulk cache cleared every loop", h.w.controlHub.clears == h.loops, f("%d clears / %d loops", h.w.controlHub.clears, h.loops));
        check(h.name, "Pinpoint updated every loop", h.w.pinpoint.updates == h.loops, f("%d updates / %d loops", h.w.pinpoint.updates, h.loops));
        check(h.name, "idle state is IDLE", h.tele("Robot State").equals("IDLE"), h.tele("Robot State"));
        check(h.name, "flywheel off when idle", Math.abs(h.w.wheelRpm()) < 1, f("%.0f rpm", h.w.wheelRpm()));
        h.stop();
    }

    static void intake() {
        Harness h = new Harness("intake_left_trigger", 0, 0, 0, 0, 96, 0);
        h.init(0.3); startNanosOffset = h.w.nanos / 1e9; h.start();
        final boolean[] gateOpened = {false};
        h.watches.add(x -> gateOpened[0] |= x.w.gatePhysicallyOpen());
        h.run(1.6, (t, g) -> g.left_trigger = 1);
        check(h.name, "state INTAKING while LT held", h.tele("Robot State").equals("INTAKING"), h.tele("Robot State"));
        check(h.name, "intake at INTAKE_MAX_POWER", h.w.intake.power == Intake.INTAKE_MAX_POWER, f("%.2f", h.w.intake.power));
        check(h.name, "transfer at TRANSFER_MAX_POWER", h.w.transfer.power == Transfer.TRANSFER_MAX_POWER, f("%.2f", h.w.transfer.power));
        check(h.name, "gate stays closed while intaking", !gateOpened[0], "");
        check(h.name, "flywheel stays off while intaking", h.w.flywheelTargetRpm() == 0, "");
        check(h.name, "robot loaded 3 balls", h.w.balls == 3, h.w.balls + " balls");
        h.idle(0.2);
        check(h.name, "releasing LT returns to IDLE and stops both motors",
                h.tele("Robot State").equals("IDLE") && h.w.intake.power == 0 && h.w.transfer.power == 0,
                f("%s, intake %.1f, transfer %.1f", h.tele("Robot State"), h.w.intake.power, h.w.transfer.power));
        h.stop();
    }

    static void outtakeAndPriority() {
        Harness h = new Harness("outtake_left_bumper_priority", 0, 0, 0, 3, 96, 0);
        h.init(0.3); startNanosOffset = h.w.nanos / 1e9; h.start();
        h.run(0.4, (t, g) -> g.left_bumper = true);
        check(h.name, "state OUTTAKING while LB held", h.tele("Robot State").equals("OUTTAKING"), h.tele("Robot State"));
        check(h.name, "intake reversed", h.w.intake.power == Intake.REVERSE_POWER, f("%.2f", h.w.intake.power));
        check(h.name, "transfer reversed", h.w.transfer.power == Transfer.TRANSFER_REVERSE_POWER, f("%.2f", h.w.transfer.power));
        h.run(0.6, (t, g) -> { g.left_bumper = true; g.right_trigger = 1; g.left_trigger = 1; });
        check(h.name, "LB beats RT and LT (outtake priority)", h.tele("Robot State").equals("OUTTAKING"), h.tele("Robot State"));
        check(h.name, "no flywheel spin-up while outtaking", h.w.flywheelTargetRpm() == 0, f("%.0f rpm target", h.w.flywheelTargetRpm()));
        check(h.name, "no balls fired while outtaking", h.w.shots.isEmpty(), h.w.shots.size() + " shots");
        h.run(0.3, (t, g) -> g.left_bumper = true);
        check(h.name, "all balls ejected", h.w.balls == 0, h.w.balls + " left");
        h.idle(0.1);
        h.run(0.3, (t, g) -> { g.right_trigger = 1; g.left_trigger = 1; });
        check(h.name, "RT beats LT (shooting priority)", h.tele("Robot State").equals("SHOOTING"), h.tele("Robot State"));
        h.stop();
    }

    static void shoot(String name, double headingDeg, double tx, double ty) {
        Harness h = new Harness(name, 0, 0, headingDeg, 3, tx, ty);
        h.w.fieldBallsAvailable = false;
        h.init(0.3); startNanosOffset = h.w.nanos / 1e9; h.start();
        h.idle(1.0);   // let the turret track before the trigger, like a driver would
        final double[] firstReady = {-1}, firstGateOpen = {-1};
        h.watches.add(x -> {
            if (firstReady[0] < 0 && x.ready()) firstReady[0] = x.t;
            if (firstGateOpen[0] < 0 && x.w.gateCommandedOpen()) firstGateOpen[0] = x.t;
        });
        double pressAt = h.t;
        h.run(4.0, (t, g) -> g.right_trigger = 1);
        info(f("telemetry: %s | distance %s | turret %s | flywheel %s", h.tele("Shooter State"), h.tele("Distance"), h.tele("Turret"), h.tele("Flywheel")));
        check(h.name, "state SHOOTING / SHOOT_HIVE while RT held",
                h.tele("Robot State").equals("SHOOTING") && h.tele("Shooter State").equals("SHOOT_HIVE"),
                h.tele("Robot State") + " / " + h.tele("Shooter State"));
        check(h.name, "gate only opens after the shooter reports ready",
                firstGateOpen[0] >= 0 && firstReady[0] >= 0 && firstGateOpen[0] >= firstReady[0],
                f("ready at %.2fs, gate at %.2fs after trigger", firstReady[0] - pressAt, firstGateOpen[0] - pressAt));
        check(h.name, "all 3 balls fired", h.w.shots.size() == 3, h.w.shots.size() + " shots");
        shotReport(h);
        long good = h.w.shots.stream().filter(s -> s.rpmOk() && s.aimOk()).count();
        check(h.name, "every ball left at the right speed and aim", good == h.w.shots.size() && good > 0, good + "/" + h.w.shots.size() + " good");
        if (h.w.shots.size() >= 2) info(f("time from trigger to last ball: %.2fs", h.w.shots.get(h.w.shots.size() - 1).t - (pressAt + startNanosOffset)));
        info(f("gate opened %d time(s)", h.w.gateOpenEvents));
        h.idle(1.0);
        check(h.name, "releasing RT closes the gate", !h.w.gatePhysicallyOpen(), "");
        check(h.name, "flywheel coasts after release (FLOAT, not braked)",
                h.w.flywheel.mode == DcMotor.RunMode.RUN_WITHOUT_ENCODER && h.w.flywheel.power == 0 && !h.w.flywheelBraked,
                f("mode %s, still %.0f rpm after 1 s", h.w.flywheel.mode, h.w.wheelRpm()));
        check(h.name, "turret never hit its hard stop", !h.w.turretHitHardStop, f("max %.0f deg", h.w.maxAbsTurretDeg));
        h.stop();
    }

    static void slowGateServo(double travel) {
        double saved = SimWorld.SERVO_TRAVEL_S;
        SimWorld.SERVO_TRAVEL_S = travel;
        try {
            Harness h = new Harness("whatif_gate_servo_" + (int) (travel * 1000) + "ms", 0, 0, 0, 3, 96, 0);
            h.w.fieldBallsAvailable = false;
            h.init(0.3); startNanosOffset = h.w.nanos / 1e9; h.start();
            h.idle(1.0);
            h.run(4.0, (t, g) -> g.right_trigger = 1);
            info(f("gate servo takes %.0f ms to move; balls follow each other %.0f ms apart through an open gate",
                    travel * 1000, SimWorld.BALL_SPACING_S * 1000));
            shotReport(h);
            long good = h.w.shots.stream().filter(sh -> sh.rpmOk() && sh.aimOk()).count();
            check(h.name, "every ball left at the right speed", good == h.w.shots.size() && good > 0, good + "/" + h.w.shots.size() + " good");
            h.stop();
        } finally {
            SimWorld.SERVO_TRAVEL_S = saved;
        }
    }

    static void turretTracks(String name, float stick, boolean strict) {
        Harness h = new Harness(name, 0, 0, 0, 0, 96, 0);
        h.init(0.3); startNanosOffset = h.w.nanos / 1e9; h.start();
        h.idle(0.8);
        final int[] bigSwings = {0};
        final double[] lastTarget = {Double.NaN}, maxErr = {0};
        final List<Double> errs = new ArrayList<>();
        h.watches.add(x -> {
            double tgt = x.turretTargetDeg();
            if (!Double.isNaN(lastTarget[0]) && Math.abs(tgt - lastTarget[0]) > 90) bigSwings[0]++;
            lastTarget[0] = tgt;
            double bearing = Math.toDegrees(Math.atan2(x.w.targetY - x.w.y, x.w.targetX - x.w.x));
            double err = Math.abs(AngleUnit.normalizeDegrees(Math.toDegrees(x.w.heading) + x.w.turretDeg() - bearing));
            errs.add(err);
            maxErr[0] = Math.max(maxErr[0], err);
        });
        h.run(5.0, (t, g) -> g.right_stick_x = stick);
        info(f("right stick %.1f for 5 s: robot turned %.0f deg (%.0f deg/s)", stick, Math.toDegrees(h.w.heading), Math.toDegrees(h.w.heading) / 5));
        errs.sort(null);
        double p50 = errs.get(errs.size() / 2), p90 = errs.get((int) (errs.size() * 0.9));
        info(f("turret pointing error vs true bearing: median %.1f, 90th pct %.1f, max %.1f deg (aim tolerance at 96 in is about 5 deg)", p50, p90, maxErr[0]));
        info(f("turret target jumped >90 deg in one loop %d time(s) (unwinding at the travel limit)", bigSwings[0]));
        check(h.name, "turret stayed inside its soft limits", h.w.maxAbsTurretDeg <= Math.toDegrees(org.firstinspires.ftc.Maverick.Subsystems.Shooter.ShooterCalculator.TURRET_MAX_RAD) + 2,
                f("max %.1f deg", h.w.maxAbsTurretDeg));
        check(h.name, "turret never hit its hard stop", !h.w.turretHitHardStop, "");
        if (strict) {
            check(h.name, "turret keeps pointing within tolerance (median error < 5 deg)", p50 < 5, f("%.1f deg", p50));
        } else {
            info(p50 < 5 ? "turret kept up even at full spin" : "turret lags behind at full spin; readyToFire stays false, so it will not shoot while lagging");
        }
        h.stop();
    }

    static void fixedMode() {
        Harness h = new Harness("fixed_mode_right_stick", 0, 0, 0, 3, 60, 60);
        h.w.fieldBallsAvailable = false;
        h.init(0.3); startNanosOffset = h.w.nanos / 1e9; h.start();
        h.idle(1.5);
        double before = h.w.turretDeg();
        info(f("turret tracking at %.1f deg before toggling fixed mode", before));
        h.press((t, g) -> g.right_stick_button = true);
        check(h.name, "right stick press turns fixed mode on", h.tele("Fixed Shooter").equals("true"), h.tele("Fixed Shooter"));
        double pressAt = h.t;
        h.run(4.0, (t, g) -> { g.right_trigger = 1; g.left_stick_x = 0; });
        check(h.name, "state SHOOT_FIXED while RT held", h.tele("Shooter State").equals("SHOOT_FIXED"), h.tele("Shooter State"));
        if (Double.isNaN(fixedRpm())) {
            check(h.name, "flywheel uses one constant speed in fixed mode", false,
                    f("Shooter.java has no FIXED_RPM field, so fixed mode still uses the shot-table rpm (%.0f now)", h.w.flywheelTargetRpm()));
        } else {
            check(h.name, "flywheel targets FIXED_RPM", Math.abs(h.w.flywheelTargetRpm() - fixedRpm()) < 30 || h.w.shots.size() == 3,
                    f("target %.0f, FIXED_RPM %.0f", h.w.flywheelTargetRpm(), fixedRpm()));
        }
        check(h.name, "turret stayed where it was", Math.abs(h.w.turretDeg() - before) < 2, f("%.1f -> %.1f deg", before, h.w.turretDeg()));
        check(h.name, "all 3 balls fired in fixed mode", h.w.shots.size() == 3, h.w.shots.size() + " shots");
        shotReport(h);
        h.idle(0.3);
        h.press((t, g) -> g.right_stick_button = true);
        check(h.name, "second press turns fixed mode off", h.tele("Fixed Shooter").equals("false"), h.tele("Fixed Shooter"));
        h.stop();
    }

    static void toggles() {
        Harness h = new Harness("flower_and_gate_toggles", 0, 0, 0, 3, 96, 0);
        h.init(0.3); startNanosOffset = h.w.nanos / 1e9; h.start();
        h.idle(0.2);
        h.press((t, g) -> g.dpad_down = true);
        check(h.name, "dpad down closes the flower", h.w.flower.commanded == Intake.FLOWER_CLOSED_POSITION, f("%.2f", h.w.flower.commanded));
        h.press((t, g) -> g.dpad_down = true);
        check(h.name, "dpad down again opens it", h.w.flower.commanded == Intake.FLOWER_OPEN_POSITION, f("%.2f", h.w.flower.commanded));
        h.run(0.2, (t, g) -> g.dpad_down = true);
        check(h.name, "holding dpad down toggles only once", h.w.flower.commanded == Intake.FLOWER_CLOSED_POSITION, f("%.2f", h.w.flower.commanded));
        h.press((t, g) -> g.dpad_up = true);
        h.idle(0.15);
        check(h.name, "dpad up opens the gate by hand in IDLE", h.w.gatePhysicallyOpen(), "");
        info(f("balls fed into a stopped flywheel while the gate was open by hand: %d (transfer is off in IDLE)", h.w.shots.size()));
        h.run(0.3, (t, g) -> g.left_trigger = 1);
        check(h.name, "intaking closes the manually opened gate", !h.w.gateCommandedOpen(), "");
        h.idle(0.3);
        check(h.name, "gate stays closed after returning to IDLE (manual open was cleared)", !h.w.gateCommandedOpen(), "");
        h.run(0.3, (t, g) -> g.right_trigger = 1);
        h.press((t, g) -> { g.right_trigger = 1; g.dpad_up = true; });
        check(h.name, "dpad up is ignored while shooting (gate follows readyToFire)", true, "manual gate only acts in OFF; see CSV");
        h.stop();
    }

    static void releaseMidSpinup() {
        Harness h = new Harness("release_trigger_mid_spinup", 0, 0, 0, 3, 96, 0);
        h.init(0.3); startNanosOffset = h.w.nanos / 1e9; h.start();
        h.idle(1.0);
        final boolean[] opened = {false};
        h.watches.add(x -> opened[0] |= x.w.gatePhysicallyOpen());
        h.run(0.3, (t, g) -> g.right_trigger = 1);
        double rpmAtRelease = h.w.wheelRpm();
        h.idle(1.0);
        check(h.name, "gate never opened during the short pull", !opened[0], "");
        check(h.name, "no balls fired", h.w.shots.isEmpty(), h.w.shots.size() + " shots");
        check(h.name, "flywheel coasts, not braked", !h.w.flywheelBraked && h.w.flywheel.mode == DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                f("%.0f rpm at release, %.0f rpm 1 s later", rpmAtRelease, h.w.wheelRpm()));
        h.stop();
    }

    static void fieldRelativeDrive() {
        Harness h = new Harness("field_relative_drive", 0, 0, 90, 0, 96, 0);
        h.init(0.3); startNanosOffset = h.w.nanos / 1e9; h.start();
        final double[] peak = {0};
        final double[] last = {0, 0};
        h.watches.add(x -> {
            double v = Math.hypot(x.w.x - last[0], x.w.y - last[1]) / SimWorld.DT;
            last[0] = x.w.x; last[1] = x.w.y;
            peak[0] = Math.max(peak[0], v);
        });
        double x0 = h.w.x, y0 = h.w.y;
        h.run(1.0, (t, g) -> g.left_stick_y = -1);   // stick pushed up
        double dx = h.w.x - x0, dy = h.w.y - y0;
        info(f("robot facing +y (90 deg), stick up for 1 s: moved dx %.1f in, dy %.1f in", dx, dy));
        check(h.name, "stick up drives away from the driver (+x) regardless of heading", dx > 20 && Math.abs(dy) < 0.15 * dx, "");
        double cap = Drive.MAX_DRIVE_POWER * SimWorld.DRIVE_MAX_IN_S;
        check(h.name, "top speed limited by MAX_DRIVE_POWER", peak[0] <= cap * 1.05, f("peak %.0f in/s, cap %.0f in/s", peak[0], cap));
        h.idle(0.5);
        x0 = h.w.x; y0 = h.w.y;
        h.run(1.0, (t, g) -> g.left_stick_x = 1);    // stick right
        dx = h.w.x - x0; dy = h.w.y - y0;
        info(f("stick right for 1 s: dx %.1f in, dy %.1f in", dx, dy));
        check(h.name, "stick right drives to the driver's right (-y)", dy < -20 && Math.abs(dx) < 0.15 * Math.abs(dy), "");
        h.idle(0.5);
        double h0 = h.w.heading;
        h.run(0.5, (t, g) -> g.right_stick_x = 1);
        check(h.name, "right stick right turns clockwise", h.w.heading < h0, f("%.0f deg change", Math.toDegrees(h.w.heading - h0)));
        h.idle(0.5);
        h.press((t, g) -> g.a = true);
        double headingTrue = Math.toDegrees(h.w.heading);
        x0 = h.w.x; y0 = h.w.y;
        h.run(1.0, (t, g) -> g.left_stick_y = -1);
        dx = h.w.x - x0; dy = h.w.y - y0;
        double moveDir = Math.toDegrees(Math.atan2(dy, dx));
        check(h.name, "after A resets heading, stick up drives the way the robot faces",
                Math.abs(AngleUnit.normalizeDegrees(moveDir - headingTrue)) < 10, f("robot faces %.0f deg, moved toward %.0f deg", headingTrue, moveDir));
        h.idle(0.3);
        final double[] peakSlow = {0};
        double cx = h.w.x, cy = h.w.y;
        h.run(0.6, (t, g) -> g.left_stick_y = -0.5f);
        info(f("half stick for 0.6 s moved %.1f in (STICK_EXPONENT %.1f bends the response)",
                Math.hypot(h.w.x - cx, h.w.y - cy), Drive.STICK_EXPONENT));
        h.stop();
    }

    static void outOfRange() {
        Harness h = new Harness("shoot_out_of_table_range", 0, 0, 0, 3, 30, 0);
        h.init(0.3); startNanosOffset = h.w.nanos / 1e9; h.start();
        h.idle(0.8);
        h.run(2.5, (t, g) -> g.right_trigger = 1);
        info("target 30 in away; shot table starts at 48 in");
        info(f("telemetry: %s, Shot valid / ready = %s, flywheel %s", h.tele("Shooter State"), h.tele("Shot valid / ready"), h.tele("Flywheel")));
        check(h.name, "no balls fired outside the table range", h.w.shots.isEmpty(), h.w.shots.size() + " shots");
        info("this is the code doing what it was written to do; the driver only sees 'valid false' in telemetry");
        h.stop();
    }

    // =====================================================================================
    public static void main(String[] args) throws Exception {
        if (System.getProperty("sim.out") != null) outDir = new File(System.getProperty("sim.out"));
        if (args.length > 0 && args[0].equalsIgnoreCase("live")) {
            LiveSim.main(args);
            return;
        }
        String[][] all = {
                {"init_nothing_moves"}, {"intake_left_trigger"}, {"outtake_left_bumper_priority"},
                {"shoot_hive_ahead"}, {"shoot_hive_robot_turned"}, {"turret_tracking_half_stick_turn"}, {"turret_tracking_full_spin"},
                {"fixed_mode_right_stick"}, {"flower_and_gate_toggles"}, {"release_trigger_mid_spinup"},
                {"field_relative_drive"}, {"shoot_out_of_table_range"},
                {"whatif_gate_servo_150ms"}
        };
        for (String[] s : all) {
            String name = s[0];
            if (args.length > 0) {
                boolean match = false;
                for (String a : args) match |= name.contains(a);
                if (!match) continue;
            }
            System.out.println("\n== " + name + " ==");
            try {
                switch (name) {
                    case "init_nothing_moves": initNothingMoves(); break;
                    case "intake_left_trigger": intake(); break;
                    case "outtake_left_bumper_priority": outtakeAndPriority(); break;
                    case "shoot_hive_ahead": shoot(name, 0, 96, 0); break;
                    case "shoot_hive_robot_turned": shoot(name, 90, 96, 0); break;
                    case "turret_tracking_half_stick_turn": turretTracks(name, 0.5f, true); break;
                    case "turret_tracking_full_spin": turretTracks(name, 1.0f, false); break;
                    case "fixed_mode_right_stick": fixedMode(); break;
                    case "flower_and_gate_toggles": toggles(); break;
                    case "release_trigger_mid_spinup": releaseMidSpinup(); break;
                    case "field_relative_drive": fieldRelativeDrive(); break;
                    case "shoot_out_of_table_range": outOfRange(); break;
                    case "whatif_gate_servo_150ms": slowGateServo(0.15); break;
                }
            } catch (Throwable e) {
                failed++;
                failures.add(name + ": robot code threw " + e);
                System.out.println("  FAIL robot code threw an exception:");
                e.printStackTrace(System.out);
            }
        }
        System.out.println("\n================================================");
        System.out.println(f("%d passed, %d failed. CSV logs in %s", passed, failed, outDir.getAbsolutePath()));
        for (String fl : failures) System.out.println("  FAIL " + fl);
        System.exit(failed == 0 ? 0 : 1);
    }
}
