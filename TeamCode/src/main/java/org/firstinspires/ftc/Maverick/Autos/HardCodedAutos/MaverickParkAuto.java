package org.firstinspires.ftc.Maverick.Autos.HardCodedAutos;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.Maverick.FieldConstants;
import org.firstinspires.ftc.Maverick.Robot;
import org.firstinspires.ftc.Maverick.Subsystems.Drive.Drive;
import org.firstinspires.ftc.Maverick.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;

/**
 * Park-only autonomous: start at (START_X, START_Y), drive straight down the field (toward y = 0) to
 * PARK_Y, and stop. Same heading as the regular start (red 90 deg: facing the far wall), so the robot
 * drives backwards. No shooting.
 *
 * All positions are RED, Pedro frame (inches). Blue is mirrored like FieldConstants:
 * (x, y, h) -> (144 - x, 144 - y, h + 180).
 *
 * Same go-to-point drive, drive signs and wrong-way safety as MaverickAuto. Keep the DRIVE SIGNS
 * below the same as MaverickAuto's.
 */
@Autonomous(name = "Maverick Auto: Park", group = "Auto", preselectTeleOp = "18121 Maverick TeleOp")
public class MaverickParkAuto extends OpMode {

    // ===== Path (RED, Pedro frame, inches) =====
    public static double START_X = 8.0;
    public static double START_Y = 131.8;
    public static double PARK_Y = 115.0;          // x stays at START_X

    // ===== Drive controller (same as MaverickAuto) =====
    public static double KP_POS = 0.06;
    public static double MAX_SPEED = 0.6;
    public static double MIN_SPEED = 0.12;
    public static double KP_HEADING = 1.5;
    public static double MAX_TURN = 0.4;
    public static double POS_TOLERANCE_IN = 1.0;
    public static double HEADING_TOLERANCE_DEG = 2.0;
    public static double SETTLE_S = 0.15;
    public static double DRIVE_TIMEOUT_S = 4.0;

    // ===== DRIVE SIGNS: keep these the same as MaverickAuto =====
    public static double FORWARD_SIGN = -1.0;
    public static double RIGHT_SIGN = -1.0;
    public static double CCW_SIGN = 1.0;

    public static double WRONG_WAY_IN = 6.0;
    public static double HEADING_ABORT_DEG = 30.0;

    private enum State { DRIVE_TO_PARK, DONE, ABORTED }

    private List<LynxModule> hubs;
    private Drive drive;
    private Shooter shooter;

    private State state = State.DRIVE_TO_PARK;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime settleTimer = new ElapsedTime();
    private boolean settling = false;
    private double minDistIn = Double.POSITIVE_INFINITY;
    private String abortReason = "";

    private FieldConstants.Alliance alliance;
    private double holdHeadingRad;
    private double goalX, goalY;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        drive = new Drive(hardwareMap);
        shooter = new Shooter(hardwareMap, drive);   // only so the turret holds home and hands off to TeleOp
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) Robot.ALLIANCE = FieldConstants.Alliance.BLUE;
        if (gamepad1.bWasPressed()) Robot.ALLIANCE = FieldConstants.Alliance.RED;
        FieldConstants.Alliance a = Robot.ALLIANCE;
        boolean red = a == FieldConstants.Alliance.RED;
        telemetry.addData("Alliance", "%s   (X = blue, B = red)", a);
        telemetry.addData("Place robot at", "(%.1f, %.1f) facing %.0f deg, turret at its zero stop",
                red ? START_X : 144.0 - START_X, red ? START_Y : 144.0 - START_Y, FieldConstants.testStartDeg(a));
        telemetry.addData("Park at", "(%.1f, %.1f)",
                red ? START_X : 144.0 - START_X, red ? PARK_Y : 144.0 - PARK_Y);
        telemetry.update();
    }

    @Override
    public void start() {
        alliance = Robot.ALLIANCE;
        holdHeadingRad = Math.toRadians(FieldConstants.testStartDeg(alliance));
        double startX = mirrorX(START_X), startY = mirrorY(START_Y);
        drive.setPose(new Pose2D(DistanceUnit.INCH, startX, startY, AngleUnit.RADIANS, holdHeadingRad));
        shooter.getTurret().zeroAt(0.0);

        goalX = mirrorX(START_X);
        goalY = mirrorY(PARK_Y);
        minDistIn = Double.POSITIVE_INFINITY;
        settling = false;
        enter(State.DRIVE_TO_PARK);
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        drive.updatePose();

        if (state == State.DRIVE_TO_PARK) {
            if (driveToGoal()) {
                drive.stop();
                enter(State.DONE);
            }
        } else {
            drive.stop();
        }

        shooter.idleShooter();   // turret stays home
        shooter.update();

        telemetry.addData("State", state == State.ABORTED ? "ABORTED: " + abortReason : state.toString());
        telemetry.addData("Goal", "(%.1f, %.1f)  dist %.1f in", goalX, goalY, distToGoal());
        drive.addTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
        // hand the pose and turret angle to TeleOp
        drive.saveForTeleop();
        shooter.getTurret().saveForTeleop();
        shooter.stop();
    }

    // ---------------------------------------------------------------------------------------------

    private double mirrorX(double redX) { return alliance == FieldConstants.Alliance.RED ? redX : 144.0 - redX; }
    private double mirrorY(double redY) { return alliance == FieldConstants.Alliance.RED ? redY : 144.0 - redY; }

    private void enter(State s) {
        state = s;
        stateTimer.reset();
    }

    private double distToGoal() {
        return Math.hypot(goalX - drive.getX(), goalY - drive.getY());
    }

    /** One step toward (goalX, goalY) holding holdHeadingRad. Returns true once there and settled. */
    private boolean driveToGoal() {
        double ex = goalX - drive.getX();
        double ey = goalY - drive.getY();
        double dist = Math.hypot(ex, ey);
        double h = drive.getHeadingRad();
        double headingErr = AngleUnit.normalizeRadians(holdHeadingRad - h);   // + = need to turn CCW

        minDistIn = Math.min(minDistIn, dist);
        if (dist > minDistIn + WRONG_WAY_IN) {
            return abort("moving AWAY from the goal: flip FORWARD_SIGN or RIGHT_SIGN");
        }
        if (Math.abs(Math.toDegrees(headingErr)) > HEADING_ABORT_DEG) {
            return abort("heading ran away: flip CCW_SIGN");
        }
        if (stateTimer.seconds() > DRIVE_TIMEOUT_S) {
            drive.stop();
            return true;   // give up and stop where it is
        }

        boolean inside = dist < POS_TOLERANCE_IN && Math.abs(Math.toDegrees(headingErr)) < HEADING_TOLERANCE_DEG;
        if (inside) {
            if (!settling) { settling = true; settleTimer.reset(); }
            drive.stop();
            return settleTimer.seconds() >= SETTLE_S;
        }
        settling = false;

        double speed = dist < POS_TOLERANCE_IN ? 0.0 : Range.clip(KP_POS * dist, MIN_SPEED, MAX_SPEED);
        double fieldVx = dist > 1e-6 ? ex / dist * speed : 0.0;
        double fieldVy = dist > 1e-6 ? ey / dist * speed : 0.0;

        // field frame -> robot frame (forward along the heading, left 90 deg CCW from it)
        double vForward = fieldVx * Math.cos(h) + fieldVy * Math.sin(h);
        double vLeft = -fieldVx * Math.sin(h) + fieldVy * Math.cos(h);
        double turnCcw = Range.clip(KP_HEADING * headingErr, -MAX_TURN, MAX_TURN);

        drive.drive(FORWARD_SIGN * vForward, RIGHT_SIGN * -vLeft, CCW_SIGN * turnCcw, 1.0);
        return false;
    }

    private boolean abort(String why) {
        drive.stop();
        abortReason = why;
        enter(State.ABORTED);
        return false;
    }
}