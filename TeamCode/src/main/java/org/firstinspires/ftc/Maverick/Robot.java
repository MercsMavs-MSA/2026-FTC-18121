package org.firstinspires.ftc.Maverick;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.Maverick.Subsystems.Drive.Drive;
import org.firstinspires.ftc.Maverick.Subsystems.Intake.Intake;
import org.firstinspires.ftc.Maverick.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.Maverick.Subsystems.Shooter.Turret;
import org.firstinspires.ftc.Maverick.Subsystems.Transfer.Transfer;
import org.firstinspires.ftc.Maverick.Subsystems.Shooter.Vision.Vision;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;

/**
 * Controls: TWO_DRIVER picks the scheme.

 * TWO_DRIVER = true (driver + operator)
 *   DRIVER (gamepad 1):   sticks drive, LT intake, LB outtake, D-pad down reset heading,
 *                         right stick press switch CELL (only when AUTO_CELL is false).
 *                         INIT: X = blue, B = red.
 *   OPERATOR (gamepad 2): RT (hold) rev up: flywheel to speed + turret aims,
 *                         Y toggle the gate open / closed,
 *                         A feed: the transfer and intake push only while A is held, so tap it to feed a little at a time,
 *                         Back switch CLOSE / FAR speed (1 rumble = CLOSE, 2 = FAR).
 * The gate opens only on the operator's Y. Driver intake / outtake closes it (and it stays closed), so
 * In-taking always stacks balls against the gate. While the operator holds A the intake runs too, and the
 * driver's LT / LB don't change anything (the gate stays open).

 * TWO_DRIVER = false (old one-driver scheme, everything on gamepad 1)
 *   sticks drive, LT intake, LB outtake, RT (hold) spin up, A (hold) feed: opens the gate, backs off, then
 *   feeds (same sequence as the autos), D-pad up manual gate toggle, Back CLOSE / FAR, D-pad down reset heading,
 *   right stick press switch CELL (only when AUTO_CELL is false).
 */
@TeleOp(name = "18121 Maverick TeleOp", group = "Robot")
public class Robot extends OpMode {

    // CONTROL SCHEME: true = driver (gamepad 1) + operator (gamepad 2), false = the old one-driver scheme.
    public static boolean TWO_DRIVER = true;

    public enum RobotState {   // one-driver scheme only
        IDLE,
        INTAKING,
        OUTTAKING,
        SPIN_UP,    // RT: flywheel and turret only; intake and transfer carry on as if RT were not pressed
        FEEDING     // A: gate open, transfer and intake push balls into the flywheel
    }
    public static FieldConstants.Alliance ALLIANCE = FieldConstants.Alliance.RED;   // pick in INIT: X = blue, B = red
    public static boolean USE_FIELD_TARGET = true;   // false: aim at TARGET_X / TARGET_Y as set (testing)

    // AUTO CELL: aim at the CELL on our half of the field (split at y = 72, between the two CELLs).
    // false = manual: right stick press switches the CELL.
    public static boolean AUTO_CELL = true;
    public static double CELL_SWITCH_DEADBAND_IN = 6.0;   // must be this far past the middle line before it switches

    // VISION KILL SWITCH: false = odometry-only aiming. If false at INIT the Limelight isn't even opened,
    // so the robot also runs with the camera unplugged or missing from the configuration.
    public static boolean USE_VISION = true;

    // Pedro frame: field center is (72, 72). Overwritten every loop while USE_FIELD_TARGET is true.
    public static double TARGET_X = FieldConstants.FIELD_CENTER;
    public static double TARGET_Y = FieldConstants.FIELD_CENTER;

    private List<LynxModule> hubs;

    private Drive drive;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Vision vision;   // null when USE_VISION was false at INIT

    private RobotState robotState = RobotState.IDLE;   // one-driver scheme only
    private FieldConstants.HiveState hiveState = FieldConstants.HiveState.A;   // picked by field half (AUTO_CELL) or Back

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        drive = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap, drive);
        if (USE_VISION) vision = new Vision(hardwareMap);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) ALLIANCE = FieldConstants.Alliance.BLUE;
        if (gamepad1.bWasPressed()) ALLIANCE = FieldConstants.Alliance.RED;
        telemetry.addData("Alliance", "%s   (X = blue, B = red)", ALLIANCE);
        telemetry.addData("Start pose", Drive.handoffValid ? "from auto" : "test start (no auto ran)");
        telemetry.update();
    }

    @Override
    public void start() {
        if (Drive.handoffValid) {
            drive.setPose(Drive.handoffPose);
            Drive.handoffValid = false;   // use an auto's pose once; a second TeleOp run starts from the test pose
        } else {
            drive.setPose(new Pose2D(DistanceUnit.INCH, FieldConstants.testStartX(ALLIANCE), FieldConstants.testStartY(ALLIANCE),
                    AngleUnit.DEGREES, FieldConstants.testStartDeg(ALLIANCE)));
        }
        drive.setDriverForward(FieldConstants.driverForwardRad(ALLIANCE));
        shooter.getTurret().zeroAt(Turret.handoffAngleRad);
        intake.start();
        transfer.start();
        gamepad1.aWasPressed();   // discard the A press from connecting the gamepad (Start + A) so it doesn't reset heading
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        drive.updatePose();

        // shared driver buttons (both schemes)
        if (gamepad1.dpadDownWasPressed()) {
            drive.resetHeading();
            if (vision != null) vision.reset();   // the old correction was for the old heading
        }
        if (gamepad1.rightStickButtonWasPressed() && !AUTO_CELL) {
            setCell(hiveState == FieldConstants.HiveState.A ? FieldConstants.HiveState.B : FieldConstants.HiveState.A);
        }

        // intake / transfer / shooter; returns true while shooting
        boolean shooting = TWO_DRIVER ? twoDriverControls() : oneDriverControls();

        drive.teleopDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, false, false);

        // Pick the CELL for our half of the field, but never in the middle of a shot.
        if (AUTO_CELL && !shooting) setCell(cellForHalf(drive.getY()));

        if (USE_FIELD_TARGET) {
            TARGET_X = FieldConstants.aimX(ALLIANCE, hiveState);
            TARGET_Y = FieldConstants.aimY(ALLIANCE, hiveState);
        }
        shooter.setTarget(TARGET_X, TARGET_Y);

        // Odometry aims; vision (if on) nudges the aim onto the up CELL's tags.
        if (USE_VISION && vision != null) {
            vision.update(drive.getX(), drive.getY(), drive.getHeadingRad(),
                    shooter.getTurret().getAngleRad(), shooter.getTurret().getVelocityRadPerSec(),
                    TARGET_X, TARGET_Y, FieldConstants.aimTagFirstId(ALLIANCE, hiveState));
            shooter.setAimCorrection(vision.getCorrectionRad());
        } else {
            shooter.setAimCorrection(0.0);
        }
        shooter.update();
        intake.update();
        transfer.update();

        telemetry.addData("Alliance / CELL", "%s / cell %s (%s)", ALLIANCE, hiveState,
                AUTO_CELL ? "auto: by field half" : "manual: right stick switches");
        telemetry.addData("Controls", TWO_DRIVER ? "driver + operator" : "one driver");
        telemetry.addData("Status", controlStatus);
        telemetry.addData("Flywheel ready", shooter.readyToFire()
                ? (TWO_DRIVER ? "YES: open gate (Y), tap A to feed" : "YES: hold A to feed") : "no");
        drive.addTelemetry(telemetry);
        shooter.addTelemetry(telemetry);
        intake.addTelemetry(telemetry);
        transfer.addTelemetry(telemetry);
        if (USE_VISION && vision != null) vision.addTelemetry(telemetry);
        else telemetry.addData("Vision", "OFF (Robot.USE_VISION)");
        telemetry.update();
    }

    private String controlStatus = "";

    /** Driver + operator scheme. Returns true while the operator is shooting. */
    private boolean twoDriverControls() {
        // ===== OPERATOR (gamepad 2) =====
        if (gamepad2.yWasPressed()) transfer.toggleGate();
        if (gamepad2.backWasPressed()) {
            shooter.toggleShotSpeed();                           // CLOSE_RPM <-> FAR_RPM (starts on CLOSE)
            gamepad2.rumbleBlips(shooter.farShot ? 2 : 1);      // 2 blips = FAR, 1 = CLOSE
        }
        boolean revUp = gamepad2.right_trigger > 0.5;
        boolean feedHeld = gamepad2.a;

        // ===== DRIVER (gamepad 1) =====
        boolean intakeHeld = gamepad1.left_trigger > 0.5;
        boolean outtakeHeld = gamepad1.left_bumper;

        // Intake: runs with the operator's feed (pushes the balls up), otherwise the driver's LT / LB
        if (feedHeld) intake.setState(Intake.State.INTAKING);
        else if (outtakeHeld) intake.setState(Intake.State.REVERSING);
        else if (intakeHeld) intake.setState(Intake.State.INTAKING);
        else intake.setState(Intake.State.OFF);

        // Transfer: the operator's feed wins, then the driver's outtake / intake
        if (feedHeld) transfer.setState(Transfer.State.TAP_FEED);
        else if (outtakeHeld) transfer.setState(Transfer.State.REVERSING);
        else if (intakeHeld) transfer.setState(Transfer.State.INTAKING);
        else transfer.setState(Transfer.State.OFF);

        // Shooter: operator's rev up
        if (revUp) shooter.startShooter();
        else shooter.idleShooter();

        controlStatus = String.format("driver %s | operator %s%s | gate %s",
                outtakeHeld ? "OUTTAKING" : intakeHeld ? "INTAKING" : "-",
                revUp ? "REV UP" : "idle", feedHeld ? " + FEEDING" : "",
                transfer.isGateOpen() ? "OPEN" : "CLOSED");
        return revUp || feedHeld;
    }

    /** The old one-driver scheme, all on gamepad 1. Returns true while spinning up or feeding. */
    private boolean oneDriverControls() {
        if (gamepad1.dpadUpWasPressed()) transfer.toggleGate();
        if (gamepad1.backWasPressed()) {
            shooter.toggleShotSpeed();                           // CLOSE_RPM <-> FAR_RPM (starts on CLOSE)
            gamepad1.rumbleBlips(shooter.farShot ? 2 : 1);      // 2 blips = FAR, 1 = CLOSE
        }

        boolean intakeHeld = gamepad1.left_trigger > 0.5;

        if (gamepad1.left_bumper) {
            robotState = RobotState.OUTTAKING;
        } else if (gamepad1.a) {
            robotState = RobotState.FEEDING;
        } else if (gamepad1.right_trigger > 0.5) {
            robotState = RobotState.SPIN_UP;
        } else if (intakeHeld) {
            robotState = RobotState.INTAKING;
        } else {
            robotState = RobotState.IDLE;
        }

        switch (robotState) {
            case INTAKING:
                intake.setState(Intake.State.INTAKING);
                transfer.setState(Transfer.State.INTAKING);
                shooter.idleShooter();
                break;
            case OUTTAKING:
                intake.setState(Intake.State.REVERSING);
                transfer.setState(Transfer.State.REVERSING);
                shooter.idleShooter();
                break;
            case SPIN_UP:
                intake.setState(intakeHeld ? Intake.State.INTAKING : Intake.State.OFF);
                transfer.setState(intakeHeld ? Transfer.State.INTAKING : Transfer.State.OFF);
                shooter.startShooter();
                break;
            case FEEDING:
                intake.setState(Intake.State.INTAKING);
                transfer.setState(Transfer.State.FEEDING);
                shooter.startShooter();
                break;
            default:
                intake.setState(Intake.State.OFF);
                transfer.setState(Transfer.State.OFF);
                shooter.idleShooter();
                break;
        }

        controlStatus = robotState.toString();
        return robotState == RobotState.SPIN_UP || robotState == RobotState.FEEDING;
    }

    /**
     * The CELL on the robot's half of the field. The two CELLs' aim points differ in y, so the dividing line
     * is halfway between them (y = 72). Inside the deadband around that line, keep the current CELL.
     */
    private FieldConstants.HiveState cellForHalf(double robotY) {
        double yA = FieldConstants.aimY(ALLIANCE, FieldConstants.HiveState.A);
        double yB = FieldConstants.aimY(ALLIANCE, FieldConstants.HiveState.B);
        double middle = (yA + yB) / 2.0;
        FieldConstants.HiveState low = yA < yB ? FieldConstants.HiveState.A : FieldConstants.HiveState.B;
        FieldConstants.HiveState high = low == FieldConstants.HiveState.A ? FieldConstants.HiveState.B : FieldConstants.HiveState.A;
        if (robotY < middle - CELL_SWITCH_DEADBAND_IN) return low;
        if (robotY > middle + CELL_SWITCH_DEADBAND_IN) return high;
        return hiveState;
    }

    private void setCell(FieldConstants.HiveState cell) {
        if (cell == hiveState) return;
        hiveState = cell;
        if (vision != null) vision.reset();   // different CELL, different tags: relearn the correction
    }

    @Override
    public void stop() {
        shooter.stop();
        intake.stop();
        transfer.stop();
        drive.stop();
        if (vision != null) vision.stop();
    }
}