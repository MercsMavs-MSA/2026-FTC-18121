package org.firstinspires.ftc.Maverick.Tuning;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.Maverick.FieldConstants;
import org.firstinspires.ftc.Maverick.Robot;
import org.firstinspires.ftc.Maverick.Subsystems.Drive.Drive;
import org.firstinspires.ftc.Maverick.Subsystems.Intake.Intake;
import org.firstinspires.ftc.Maverick.Subsystems.Shooter.Flywheel;
import org.firstinspires.ftc.Maverick.Subsystems.Shooter.ShooterCalculator;
import org.firstinspires.ftc.Maverick.Subsystems.Shooter.Turret;
import org.firstinspires.ftc.Maverick.Subsystems.Transfer.Transfer;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.TreeMap;

/**
 * Shot map calibration: find the flywheel rpm that scores at each distance, and get rows you can
 * paste straight into ShooterCalculator.SHOT_TABLE.

 * The turret aims itself (ShooterCalculator + Pinpoint, same as Robot.java). Feeding is manual: RT opens
 * the gate and feeds whenever it is held. Wait for "Ready: YES" before feeding so each logged shot is a
 * fair test of the rpm.
 * Distance is the turret-pivot-to-target distance the shot table uses: from odometry by default, or a
 * tape-measured value if odometry isn't trusted yet.

 * SETUP (Pedro frame, see FieldConstants): pick the alliance in INIT (X = blue, B = red). Put the robot
 * at that alliance's test start pose with the turret at its zero stop (pointing backward). The target
 * is the alliance's CELL A aim point.

 * WORKFLOW at each spot:
 *   1. Drive to a spot (slow drive is always on here). Check the distance in telemetry.
 *   2. A spins the flywheel up. Set an rpm with D-pad up/down.
 *   3. When Ready shows YES, hold RT to feed.
 *   4. After each ball: X = landed SHORT, Y = GOOD (scored), B = LONG.
 *   5. Change rpm and repeat until a spot has both a SHORT and a LONG, or a few GOODs. Then move.
 *   Suggested rows appear in telemetry as "{ distance, rpm }," lines.

 * CONTROLS (gamepad1)
 *   Sticks          drive (field-relative, slow)        LT intake   LB outtake   RT feed (manual)
 *   A               flywheel on / off                   D-pad up/down  rpm +/- step   Back  step 25 <-> 100
 *   X / Y / B       log SHORT / GOOD / LONG             Right stick press  undo last log
 *   Left stick press  distance source: odometry <-> manual      D-pad left/right  manual distance -/+ 3 in

 * Results: /sdcard/FIRST/shot_calibration.csv on the Control Hub and the Robot log (tag ShotCal).
 */
@TeleOp(name = "Shot Calibration", group = "Tuning")
public class ShotCalibration extends OpMode {

    public static double START_RPM = 2400.0;
    /** A shot = wheel falls this many rpm below the speed it was holding. Lower it if shots are missed. */
    public static double DIP_RPM = 120.0;
    /** Shots within this many inches of each other are grouped into one table row. */
    public static double DISTANCE_BUCKET_IN = 3.0;

    private static final String CSV_PATH = "/sdcard/FIRST/shot_calibration.csv";
    private static final String CSV_HEADER = "seconds,distance_in,source,target_rpm,rpm_before_shot,shot_detected,result";

    private enum Result { SHORT, GOOD, LONG }

    private static class Row {
        double distIn, targetRpm, rpmBefore;
        boolean shotDetected, manual;
        Result result;
    }

    private List<LynxModule> hubs;
    private Drive drive;
    private Flywheel flywheel;
    private Turret turret;
    private Intake intake;
    private Transfer transfer;

    private final List<Row> rows = new ArrayList<>();
    private String csvError = null;

    private boolean flywheelOn = false;
    private double targetRpm = START_RPM;
    private double rpmStep = 25;
    private boolean manualDistance = false;
    private double manualDistanceIn = 96;

    private ShooterCalculator.ShotSolution shot;
    private double distanceIn = 0;
    private double targetX, targetY;

    // shot detection
    private boolean armed = false;
    private double heldRpm = 0, heldTarget = 0, lastTarget = 0;
    private int shotsSeen = 0;
    private boolean shotPending = false;
    private double shotDist, shotTarget, shotRpmBefore;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        drive = new Drive(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);

    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) Robot.ALLIANCE = FieldConstants.Alliance.BLUE;
        if (gamepad1.bWasPressed()) Robot.ALLIANCE = FieldConstants.Alliance.RED;
        FieldConstants.Alliance a = Robot.ALLIANCE;
        telemetry.addData("Alliance", "%s   (X = blue, B = red)", a);
        telemetry.addData("Place robot at", "(%.1f, %.1f, %.0f deg), turret at its zero stop",
                FieldConstants.testStartX(a), FieldConstants.testStartY(a), FieldConstants.testStartDeg(a));
        telemetry.addData("Target (CELL A)", "(%.1f, %.1f)",
                FieldConstants.aimX(a, FieldConstants.HiveState.A), FieldConstants.aimY(a, FieldConstants.HiveState.A));
        telemetry.update();
    }

    @Override
    public void start() {
        FieldConstants.Alliance a = Robot.ALLIANCE;
        targetX = FieldConstants.aimX(a, FieldConstants.HiveState.A);
        targetY = FieldConstants.aimY(a, FieldConstants.HiveState.A);
        drive.setPose(new Pose2D(DistanceUnit.INCH, FieldConstants.testStartX(a), FieldConstants.testStartY(a),
                AngleUnit.DEGREES, FieldConstants.testStartDeg(a)));
        drive.setDriverForward(FieldConstants.driverForwardRad(a));
        turret.zeroAt(0);
        intake.start();
        transfer.start();
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        drive.updatePose();

        // --- buttons ---
        if (gamepad1.aWasPressed()) flywheelOn = !flywheelOn;
        if (gamepad1.dpadUpWasPressed()) targetRpm += rpmStep;
        if (gamepad1.dpadDownWasPressed()) targetRpm -= rpmStep;
        targetRpm = Math.max(0, Math.min(Flywheel.MAX_RPM, targetRpm));
        if (gamepad1.backWasPressed()) rpmStep = (rpmStep == 25) ? 100 : 25;
        if (gamepad1.leftStickButtonWasPressed()) manualDistance = !manualDistance;
        if (gamepad1.dpadRightWasPressed()) manualDistanceIn += 3;
        if (gamepad1.dpadLeftWasPressed()) manualDistanceIn = Math.max(0, manualDistanceIn - 3);

        // --- drive (slow, field-relative) ---
        drive.teleopDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, false, true);

        // --- aim: same calculator as Robot.java; only the turret angle and distance are used ---
        shot = ShooterCalculator.solve(drive.getX(), drive.getY(), drive.getHeadingRad(),
                targetX, targetY, turret.getAngleRad(), 0.0, 0.0);
        distanceIn = manualDistance ? manualDistanceIn : shot.distanceIn;
        turret.setTargetAngle(shot.turretAngleRad);
        turret.update();

        // --- flywheel at the rpm being tested ---
        flywheel.setTargetRpm(flywheelOn ? targetRpm : 0);
        flywheel.update();
        trackShots();

        // --- intake / transfer: feeding is manual, "ready" is only a hint for the driver ---
        boolean aimed = manualDistance || ShooterCalculator.isAimed(shot, turret.getAngleRad());
        boolean ready = flywheelOn && flywheel.atSpeed() && aimed;
        if (gamepad1.left_bumper) {
            intake.setState(Intake.State.REVERSING);
            transfer.setState(Transfer.State.REVERSING);
        } else if (gamepad1.right_trigger > 0.5) {
            intake.setState(Intake.State.INTAKING);
            transfer.setState(Transfer.State.FEEDING);
        } else if (gamepad1.left_trigger > 0.5) {
            intake.setState(Intake.State.INTAKING);
            transfer.setState(Transfer.State.INTAKING);
        } else {
            intake.setState(Intake.State.OFF);
            transfer.setState(Transfer.State.OFF);
        }
        intake.update();
        transfer.update();

        // --- logging ---
        if (gamepad1.xWasPressed()) logResult(Result.SHORT);
        if (gamepad1.yWasPressed()) logResult(Result.GOOD);
        if (gamepad1.bWasPressed()) logResult(Result.LONG);
        if (gamepad1.rightStickButtonWasPressed()) undo();

        // --- telemetry ---
        telemetry.addData("Distance", "%.1f in (%s)", distanceIn, manualDistance ? "MANUAL, L-stick for odometry" : "odometry");
        telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f deg", drive.getX(), drive.getY(), Math.toDegrees(drive.getHeadingRad()));
        telemetry.addData("Flywheel", "%.0f / %.0f rpm (step %.0f) %s", flywheel.getRpm(), targetRpm, rpmStep, flywheelOn ? "ON" : "off");
        telemetry.addData("Ready", ready ? "YES: hold RT to feed"
                : !flywheelOn ? "no: flywheel off (A)" : !flywheel.atSpeed() ? "no: spinning up" : "no: turret not aimed");
        telemetry.addData("Turret", "%.1f / %.1f deg%s", Math.toDegrees(turret.getAngleRad()), Math.toDegrees(shot.turretAngleRad),
                shot.reachable ? "" : " (target outside turret travel)");
        telemetry.addData("Shots / logged", "%d / %d", shotsSeen, rows.size());
        telemetry.addData("Pending shot", shotPending
                ? String.format(Locale.US, "%.0f rpm at %.0f in: press X short / Y good / B long", shotTarget, shotDist)
                : "none");
        if (!rows.isEmpty()) {
            Row last = rows.get(rows.size() - 1);
            telemetry.addData("Last logged", "%s  %.0f in @ %.0f rpm", last.result, last.distIn, last.targetRpm);
        }
        List<String> summary = summaryLines();
        if (!summary.isEmpty()) telemetry.addLine("--- paste into SHOT_TABLE ---");
        for (int i = Math.max(0, summary.size() - 6); i < summary.size(); i++) telemetry.addLine(summary.get(i));
        if (csvError != null) telemetry.addData("CSV write failed", csvError);
        telemetry.update();
    }

    @Override
    public void stop() {
        flywheel.stop();
        turret.stop();
        intake.stop();
        transfer.stop();
        drive.stop();
        for (String line : summaryLines()) {
            RobotLog.ii("ShotCal", "TABLE " + line);
            appendCsv("# " + line);
        }
    }

    /** A shot = the wheel falls DIP_RPM below the speed it was holding. Remembers the speed and distance just before. */
    private void trackShots() {
        double target = flywheel.getTargetRpm();
        if (target <= 0 || target != lastTarget) armed = false;   // a retarget is not a shot
        lastTarget = target;
        if (target <= 0) return;

        if (flywheel.atSpeed()) {
            armed = true;
            heldRpm = flywheel.getRpm();
            heldTarget = target;
        } else if (armed && flywheel.getRpm() < heldTarget - DIP_RPM) {
            shotsSeen++;
            shotPending = true;
            shotDist = distanceIn;
            shotTarget = heldTarget;
            shotRpmBefore = heldRpm;
            armed = false;
        }
    }

    private void logResult(Result result) {
        Row r = new Row();
        r.result = result;
        r.manual = manualDistance;
        if (shotPending) {
            r.distIn = shotDist;
            r.targetRpm = shotTarget;
            r.rpmBefore = shotRpmBefore;
            r.shotDetected = true;
            shotPending = false;
        } else {
            r.distIn = distanceIn;
            r.targetRpm = targetRpm;
            r.rpmBefore = flywheel.getRpm();
            r.shotDetected = false;
        }
        rows.add(r);
        String line = String.format(Locale.US, "%.1f,%.1f,%s,%.0f,%.0f,%b,%s",
                getRuntime(), r.distIn, r.manual ? "manual" : "odometry", r.targetRpm, r.rpmBefore, r.shotDetected, r.result);
        RobotLog.ii("ShotCal", line);
        appendCsv(line);
    }

    private void undo() {
        if (rows.isEmpty()) return;
        rows.remove(rows.size() - 1);
        String line = String.format(Locale.US, "%.1f,UNDO", getRuntime());
        RobotLog.ii("ShotCal", line);
        appendCsv(line);
    }

    /**
     * One line per distance bucket, ready to paste: midpoint between the highest SHORT rpm and the lowest
     * LONG rpm, or the average GOOD rpm if there is no bracket yet.
     */
    private List<String> summaryLines() {
        TreeMap<Long, List<Row>> buckets = new TreeMap<>();
        for (Row r : rows) {
            long key = Math.round(r.distIn / DISTANCE_BUCKET_IN);
            List<Row> list = buckets.get(key);
            if (list == null) { list = new ArrayList<>(); buckets.put(key, list); }
            list.add(r);
        }
        List<String> lines = new ArrayList<>();
        for (Map.Entry<Long, List<Row>> e : buckets.entrySet()) {
            double maxShort = Double.NaN, minLong = Double.NaN, goodSum = 0, distSum = 0;
            int goodN = 0;
            for (Row r : e.getValue()) {
                distSum += r.distIn;
                if (r.result == Result.SHORT) maxShort = Double.isNaN(maxShort) ? r.targetRpm : Math.max(maxShort, r.targetRpm);
                else if (r.result == Result.LONG) minLong = Double.isNaN(minLong) ? r.targetRpm : Math.min(minLong, r.targetRpm);
                else { goodSum += r.targetRpm; goodN++; }
            }
            double dist = distSum / e.getValue().size();
            String note;
            double rpm;
            if (!Double.isNaN(maxShort) && !Double.isNaN(minLong)) {
                if (maxShort >= minLong) {
                    lines.add(String.format(Locale.US, "// %.0f in: CONFLICT, short at %.0f >= long at %.0f: reshoot", dist, maxShort, minLong));
                    continue;
                }
                rpm = (maxShort + minLong) / 2.0;
                note = String.format(Locale.US, "short<=%.0f long>=%.0f", maxShort, minLong);
            } else if (goodN > 0) {
                rpm = goodSum / goodN;
                note = goodN + " good, no bracket yet";
            } else {
                lines.add(String.format(Locale.US, "// %.0f in: need a GOOD, or both a SHORT and a LONG", dist));
                continue;
            }
            lines.add(String.format(Locale.US, "{ %.0f, %.0f },   // %s", dist, Math.round(rpm / 5.0) * 5.0, note));
        }
        return lines;
    }

    private void appendCsv(String line) {
        try {
            File f = new File(CSV_PATH);
            File dir = f.getParentFile();
            if (dir != null && !dir.exists()) dir.mkdirs();
            boolean newFile = !f.exists() || f.length() == 0;
            try (FileWriter w = new FileWriter(f, true)) {
                if (newFile) w.write(CSV_HEADER + "\n");
                w.write(line + "\n");
            }
        } catch (Exception ex) {
            csvError = String.valueOf(ex.getMessage());
        }
    }
}