package TestCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Test Intake Shooter Transfer Speed", group = "Test")
public class testing_intake_shooter_transfer_speed extends OpMode {

    // Subsystem Motors (Intake, Transfer, Shooter)
    DcMotorEx shooter;
    DcMotorEx transfer;
    DcMotor intake;

    // Max power settings matching TELE_RoosterDrive_v1
    public static double INTAKE_MAX_POWER = -1.0;
    public static double TRANSFER_MAX_POWER = -1.0;
    public static double SHOOTER_MAX_POWER = -1.0;

    @Override
    public void init() {
        // Shooter motor initialization with hardware map fallback names
        try { shooter = hardwareMap.get(DcMotorEx.class, "shooterMotor"); } catch (Exception e1) {
            try { shooter = hardwareMap.get(DcMotorEx.class, "BottomShooter"); } catch (Exception e2) {
                try { shooter = hardwareMap.get(DcMotorEx.class, "shooter"); } catch (Exception e3) {
                    shooter = null;
                }
            }
        }

        // Transfer motor initialization with hardware map fallback names
        try { transfer = hardwareMap.get(DcMotorEx.class, "transferMotor"); } catch (Exception e1) {
            try { transfer = hardwareMap.get(DcMotorEx.class, "TopShooter"); } catch (Exception e2) {
                try { transfer = hardwareMap.get(DcMotorEx.class, "transfer"); } catch (Exception e3) {
                    transfer = null;
                }
            }
        }

        // Intake motor initialization with hardware map fallback names
        try { intake = hardwareMap.get(DcMotor.class, "intakeMotor"); } catch (Exception e1) {
            try { intake = hardwareMap.get(DcMotor.class, "Intake"); } catch (Exception e2) {
                try { intake = hardwareMap.get(DcMotor.class, "intake"); } catch (Exception e3) {
                    intake = null;
                }
            }
        }

        // Reverse transfer motor as configured in TELE_RoosterDrive_v1
        if (transfer != null) {
            transfer.setDirection(DcMotor.Direction.REVERSE);
        }

        // Set run mode to RUN_WITHOUT_ENCODER for maximum raw speed / voltage
        if (shooter != null) shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (transfer != null) transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (intake != null) intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized Successfully");
        telemetry.addData("Shooter Status", shooter != null ? "Found" : "NOT FOUND");
        telemetry.addData("Transfer Status", transfer != null ? "Found" : "NOT FOUND");
        telemetry.addData("Intake Status", intake != null ? "Found" : "NOT FOUND");
    }

    @Override
    public void loop() {
        // Right trigger check on Gamepad 1 or Gamepad 2
        boolean rightTriggerPressed = gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1;

        if (rightTriggerPressed) {
            // Run intake, transfer, and shooter at max speed
            if (shooter != null) shooter.setPower(SHOOTER_MAX_POWER);
            if (transfer != null) transfer.setPower(TRANSFER_MAX_POWER);
            if (intake != null) intake.setPower(INTAKE_MAX_POWER);
        } else {
            // Stop all motors when right trigger is not pressed
            if (shooter != null) shooter.setPower(0.0);
            if (transfer != null) transfer.setPower(0.0);
            if (intake != null) intake.setPower(0.0);
        }

        // Telemetry feedback
        telemetry.addLine("=== CONTROLLER CONTROLS ===");
        telemetry.addLine("Right Trigger: Hold to run Intake, Transfer, and Shooter at MAX speed");
        telemetry.addLine("=============================");
        telemetry.addData("Right Trigger Pressed", rightTriggerPressed);
        telemetry.addData("Shooter Power", shooter != null ? shooter.getPower() : 0.0);
        telemetry.addData("Transfer Power", transfer != null ? transfer.getPower() : 0.0);
        telemetry.addData("Intake Power", intake != null ? intake.getPower() : 0.0);
        if (shooter != null) {
            telemetry.addData("Shooter Velocity (ticks/sec)", shooter.getVelocity());
        }
    }
}
