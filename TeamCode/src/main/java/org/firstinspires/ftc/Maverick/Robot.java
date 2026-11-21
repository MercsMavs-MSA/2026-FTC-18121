package org.firstinspires.ftc.Maverick;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.Maverick.Subsystems.Drive.Drive;
import org.firstinspires.ftc.Maverick.Subsystems.Intake.Intake;
import org.firstinspires.ftc.Maverick.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.Maverick.Subsystems.Shooter.Turret;
import org.firstinspires.ftc.Maverick.Subsystems.Transfer.Transfer;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;

@TeleOp(name = "18121 Maverick TeleOp", group = "Robot")
public class Robot extends OpMode {

    public enum RobotState {
        IDLE,
        INTAKING,
        OUTTAKING,
        SPIN_UP,    // RT: flywheel and turret only; intake and transfer carry on as if RT were not pressed
        FEEDING     // X: gate open, transfer and intake push balls into the flywheel
    }
    public static FieldConstants.Alliance ALLIANCE = FieldConstants.Alliance.RED;   // pick in INIT: X = blue, B = red
    public static boolean USE_FIELD_TARGET = true;   // false: aim at TARGET_X / TARGET_Y as set (testing)

    // Pedro frame: field center is (72, 72). Overwritten every loop while USE_FIELD_TARGET is true.
    public static double TARGET_X = FieldConstants.FIELD_CENTER;
    public static double TARGET_Y = FieldConstants.FIELD_CENTER;

    private List<LynxModule> hubs;

    private Drive drive;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;

    private RobotState robotState = RobotState.IDLE;
    private FieldConstants.HiveState hiveState = FieldConstants.HiveState.A;   // Back toggles after a tip

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
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        drive.updatePose();

        if (gamepad1.dpadDownWasPressed()) intake.toggleFlower();
        if (gamepad1.dpadUpWasPressed()) transfer.toggleGate();
        if (gamepad1.rightStickButtonWasPressed()) shooter.fixedShoot = !shooter.fixedShoot;
        if (gamepad1.aWasPressed()) drive.resetHeading();
        if (gamepad1.backWasPressed()) {
            hiveState = (hiveState == FieldConstants.HiveState.A) ? FieldConstants.HiveState.B : FieldConstants.HiveState.A;
        }

        boolean intakeHeld = gamepad1.left_trigger > 0.5;

        if (gamepad1.left_bumper) {
            robotState = RobotState.OUTTAKING;
        } else if (gamepad1.x) {
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

        drive.teleopDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, false, false);

        if (USE_FIELD_TARGET) {
            TARGET_X = FieldConstants.aimX(ALLIANCE, hiveState);
            TARGET_Y = FieldConstants.aimY(ALLIANCE, hiveState);
        }
        shooter.setTarget(TARGET_X, TARGET_Y);
        shooter.update();
        intake.update();
        transfer.update();

        telemetry.addData("Alliance / HIVE", "%s / cell %s up (Back toggles)", ALLIANCE, hiveState);
        telemetry.addData("Robot State", robotState);
        telemetry.addData("Fixed Shooter", shooter.fixedShoot);
        telemetry.addData("Flywheel ready", shooter.readyToFire() ? "YES: hold X to feed" : "no");
        drive.addTelemetry(telemetry);
        shooter.addTelemetry(telemetry);
        intake.addTelemetry(telemetry);
        transfer.addTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.stop();
        intake.stop();
        transfer.stop();
        drive.stop();
    }
}