package org.firstinspires.ftc.Maverick.Autos.HardCodedAutos;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.Maverick.FieldConstants;
import org.firstinspires.ftc.Maverick.Robot;
import org.firstinspires.ftc.Maverick.Subsystems.Drive.Drive;
import org.firstinspires.ftc.Maverick.Subsystems.Intake.Intake;
import org.firstinspires.ftc.Maverick.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.Maverick.Subsystems.Shooter.Vision.Vision;
import org.firstinspires.ftc.Maverick.Subsystems.Transfer.Transfer;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;

/**
 * Corner autonomous, no path planner (same start corner as MaverickParkAuto):
 *   0. Wait START_DELAY_S after Start.
 *   1. From (START_X, START_Y), drive to the shooting spot (SHOOT_X, SHOOT_Y), same heading.
 *   2. Aim at the HIVE's starting CELL (A) and spin up, then feed for FEED_TIME_S.
 *   3. Drive to the park spot (PARK_X, PARK_Y).
 * Heading for the whole auto = the regular start heading from FieldConstants (red 90 deg).
 *
 * All positions are RED, Pedro frame (inches). Blue is the same path turned half a turn about the field
 * center, like FieldConstants: (x, y, h) -> (144 - x, 144 - y, h + 180).
 *
 * Driving: a "go to point" P controller. Every loop it takes the field-frame error to the target,
 * turns it into robot-frame forward / left, drives along that line (so sideways drift is corrected
 * automatically), and holds the heading.
 *
 * BEFORE THE FIRST RUN: check the three DRIVE SIGN constants (see below). If a sign is wrong the robot
 * moves away from its target; the auto notices, stops all motors, and says which sign to flip.
 */
@Autonomous(name = "Maverick Auto: Corner Shoot + Park", group = "Auto", preselectTeleOp = "18121 Maverick TeleOp")
public class MaverickCornerAuto extends OpMode {

    // ===== Path (RED, Pedro frame, inches) =====
    public static double START_X = 8.0;
    public static double START_Y = 131.8;
    public static double SHOOT_X = 45.0;
    public static double SHOOT_Y = 117.0;
    public static double PARK_X = 12.0;           // off the wall: parking doesn't count if the robot touches it
    public static double PARK_Y = 115.0;

    // Wait this long after Start before moving (0 = go right away).
    public static double START_DELAY_S = 10.0;
    // heading for the whole auto = the start heading from FieldConstants (red 90 deg: facing the far wall)

    // The shot from here is ~63 in, about 12 in longer than from the regular auto's shooting spot.
    // true = shoot at Shooter.FAR_RPM instead of CLOSE_RPM.
    public static boolean FAR_SHOT = false;

    // ===== Shooting =====
    public static double SPINUP_TIMEOUT_S = 3.0;  // feed anyway after this long, even if not "ready"
    public static double AIM_TOLERANCE_DEG = 2.0; // turret within this of its aim before feeding
    public static double FEED_TIME_S = 5.5;       // feed this long, then leave: long enough for the whole preload

    // ===== Drive controller =====
    public static double KP_POS = 0.06;           // power per inch of distance
    public static double MAX_SPEED = 0.6;
    public static double MIN_SPEED = 0.12;        // enough to keep moving near the end (overcomes friction)
    public static double KP_HEADING = 1.5;        // power per radian of heading error
    public static double MAX_TURN = 0.4;
    public static double POS_TOLERANCE_IN = 1.0;
    public static double HEADING_TOLERANCE_DEG = 2.0;
    public static double SETTLE_S = 0.15;         // must stay inside tolerance this long
    public static double DRIVE_TIMEOUT_S = 6.0;       // per leg; both legs here are ~40 in

    // ===== DRIVE SIGNS: match how Drive.drive() moves YOUR robot =====
    // Taken from Robot.java's teleop mapping (forward = left_stick_y, right = -left_stick_x,
    // rotate = -right_stick_x), which drives correctly on the robot:
    //   drive(forward = -1) moves the robot FORWARD, drive(right = -1) moves it RIGHT,
    //   drive(rotate = -1) turns it clockwise.
    public static double FORWARD_SIGN = -1.0;     // drive() forward value for "move forward"
    public static double RIGHT_SIGN = -1.0;       // drive() right value for "move right"
    public static double CCW_SIGN = 1.0;          // drive() rotate value for "turn counter-clockwise"

    // Safety: stop everything if the robot ends up this much farther from the target than its closest
    // point so far (a wrong sign), or the heading drifts this far.
    public static double WRONG_WAY_IN = 6.0;
    public static double HEADING_ABORT_DEG = 30.0;

    private enum State { WAIT, DRIVE_TO_SHOT, SPIN_UP, FEED, DRIVE_TO_PARK, DONE, ABORTED }

    private List<LynxModule> hubs;
    private Drive drive;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Vision vision;

    private State state = State.DRIVE_TO_SHOT;
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
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap, drive);
        if (Robot.USE_VISION) vision = new Vision(hardwareMap);
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
        telemetry.addData("Shoot from", "(%.1f, %.1f) at %s speed",
                red ? SHOOT_X : 144.0 - SHOOT_X, red ? SHOOT_Y : 144.0 - SHOOT_Y, FAR_SHOT ? "FAR" : "CLOSE");
        telemetry.addData("Park at", "(%.1f, %.1f)", red ? PARK_X : 144.0 - PARK_X, red ? PARK_Y : 144.0 - PARK_Y);
        telemetry.addData("Start delay", "%.1f s", START_DELAY_S);
        telemetry.addData("Feed time", "%.1f s", FEED_TIME_S);
        telemetry.update();
    }

    @Override
    public void start() {
        alliance = Robot.ALLIANCE;
        boolean red = alliance == FieldConstants.Alliance.RED;
        double startX = red ? START_X : 144.0 - START_X;
        double startY = red ? START_Y : 144.0 - START_Y;
        holdHeadingRad = Math.toRadians(FieldConstants.testStartDeg(alliance));
        drive.setPose(new Pose2D(DistanceUnit.INCH, startX, startY, AngleUnit.RADIANS, holdHeadingRad));
        shooter.getTurret().zeroAt(0.0);
        shooter.farShot = FAR_SHOT;
        intake.start();
        transfer.start();

        setGoal(SHOOT_X, SHOOT_Y);
        enter(START_DELAY_S > 0 ? State.WAIT : State.DRIVE_TO_SHOT);
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        drive.updatePose();

        double aimX = FieldConstants.aimX(alliance, FieldConstants.HiveState.A);
        double aimY = FieldConstants.aimY(alliance, FieldConstants.HiveState.A);
        shooter.setTarget(aimX, aimY);
        if (vision != null && Robot.USE_VISION) {
            vision.update(drive.getX(), drive.getY(), drive.getHeadingRad(),
                    shooter.getTurret().getAngleRad(), shooter.getTurret().getVelocityRadPerSec(),
                    aimX, aimY, FieldConstants.aimTagFirstId(alliance, FieldConstants.HiveState.A));
            shooter.setAimCorrection(vision.getCorrectionRad());
        } else {
            shooter.setAimCorrection(0.0);
        }

        switch (state) {
            case WAIT:
                drive.stop();
                shooter.idleShooter();                   // turret stays home, flywheel on its idle roll
                ballPathOff();
                if (stateTimer.seconds() >= START_DELAY_S) enter(State.DRIVE_TO_SHOT);
                break;

            case DRIVE_TO_SHOT:
                shooter.startShooter();                  // spin up and aim while driving: saves time
                ballPathOff();
                if (driveToGoal()) {
                    drive.stop();
                    enter(State.SPIN_UP);
                }
                break;

            case SPIN_UP:
                drive.stop();
                shooter.startShooter();
                ballPathOff();
                boolean aimed = shooter.getTurret().atTarget(Math.toRadians(AIM_TOLERANCE_DEG));
                if ((shooter.readyToFire() && aimed) || stateTimer.seconds() > SPINUP_TIMEOUT_S) {
                    enter(State.FEED);
                }
                break;

            case FEED:
                drive.stop();
                shooter.startShooter();
                intake.setState(Intake.State.INTAKING);
                transfer.setState(Transfer.State.FEEDING);
                if (stateTimer.seconds() >= FEED_TIME_S) {
                    ballPathOff();
                    shooter.idleShooter();               // turret parks home, flywheel back to its idle roll
                    setGoal(PARK_X, PARK_Y);
                    enter(State.DRIVE_TO_PARK);
                }
                break;

            case DRIVE_TO_PARK:
                shooter.idleShooter();
                ballPathOff();
                if (driveToGoal()) {
                    drive.stop();
                    enter(State.DONE);
                }
                break;

            case DONE:
            case ABORTED:
            default:
                drive.stop();
                shooter.idleShooter();
                ballPathOff();
                break;
        }

        shooter.update();
        intake.update();
        transfer.update();

        telemetry.addData("State", state == State.ABORTED ? "ABORTED: " + abortReason : state.toString());
        telemetry.addData("Goal", "(%.1f, %.1f)  dist %.1f in", goalX, goalY, distToGoal());
        if (state == State.WAIT) telemetry.addData("Starting in", "%.1f s", START_DELAY_S - stateTimer.seconds());
        if (state == State.FEED) telemetry.addData("Feeding", "%.1f of %.1f s", stateTimer.seconds(), FEED_TIME_S);
        drive.addTelemetry(telemetry);
        shooter.addTelemetry(telemetry);
        if (vision != null && Robot.USE_VISION) vision.addTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
        // hand the pose and turret angle to TeleOp
        drive.saveForTeleop();
        shooter.getTurret().saveForTeleop();
        shooter.stop();
        intake.stop();
        transfer.stop();
        if (vision != null) vision.stop();
    }

    // ---------------------------------------------------------------------------------------------

    /** Set the next target. Takes RED coordinates and mirrors them for blue. */
    private void setGoal(double redX, double redY) {
        goalX = alliance == FieldConstants.Alliance.RED ? redX : 144.0 - redX;
        goalY = alliance == FieldConstants.Alliance.RED ? redY : 144.0 - redY;
        minDistIn = Double.POSITIVE_INFINITY;
        settling = false;
    }

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

        // Safety: a wrong sign drives the robot away from the goal, or spins it.
        minDistIn = Math.min(minDistIn, dist);
        if (dist > minDistIn + WRONG_WAY_IN) {
            return abort("moving AWAY from the goal: flip FORWARD_SIGN or RIGHT_SIGN (see telemetry Pose)");
        }
        if (Math.abs(Math.toDegrees(headingErr)) > HEADING_ABORT_DEG) {
            return abort("heading ran away: flip CCW_SIGN");
        }
        if (stateTimer.seconds() > DRIVE_TIMEOUT_S) {
            drive.stop();
            return true;   // give up on this leg and carry on
        }

        boolean inside = dist < POS_TOLERANCE_IN && Math.abs(Math.toDegrees(headingErr)) < HEADING_TOLERANCE_DEG;
        if (inside) {
            if (!settling) { settling = true; settleTimer.reset(); }
            drive.stop();
            return settleTimer.seconds() >= SETTLE_S;
        }
        settling = false;

        // speed along the line to the goal
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

    private void ballPathOff() {
        intake.setState(Intake.State.OFF);
        transfer.setState(Transfer.State.OFF);
    }
}