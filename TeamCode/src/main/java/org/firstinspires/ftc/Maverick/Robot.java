package org.firstinspires.ftc.Maverick;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.Maverick.Subsystems.Drive.Drive;
import org.firstinspires.ftc.Maverick.Subsystems.Intake.Intake;
import org.firstinspires.ftc.Maverick.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.Maverick.Subsystems.Shooter.Turret;
import org.firstinspires.ftc.Maverick.Subsystems.Transfer.Transfer;

import java.util.List;

@TeleOp(name = "18121 Rooster Field Relative Drive", group = "Robot")
public class Robot extends OpMode {

    public enum RobotState {
        IDLE,
        INTAKING,
        OUTTAKING,
        SHOOTING
    }
    //TODO: Add in the target positions.
    public static double TARGET_X = 0.0;
    public static double TARGET_Y = 0.0;

    private List<LynxModule> hubs;

    private Drive drive;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;

    private RobotState robotState = RobotState.IDLE;

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
    public void start() {
        drive.setPose(Drive.handoffPose);
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

        if (gamepad1.left_bumper) {
            robotState = RobotState.OUTTAKING;
        } else if (gamepad1.right_trigger > 0.5) {
            robotState = RobotState.SHOOTING;
        } else if (gamepad1.left_trigger > 0.5) {
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
            case SHOOTING:
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

        shooter.setTarget(TARGET_X, TARGET_Y);
        shooter.update();
        transfer.setFireAllowed(shooter.readyToFire());

        intake.update();
        transfer.update();

        telemetry.addData("Robot State", robotState);
        telemetry.addData("Fixed Shooter", shooter.fixedShoot);
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