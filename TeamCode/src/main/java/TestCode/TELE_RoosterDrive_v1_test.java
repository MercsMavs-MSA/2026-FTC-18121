package TestCode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;


import com.qualcomm.hardware.limelightvision.Limelight3A;

//http://limelight.local:5801 is the limelight local hosted server

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "18121 Rooster Field Relative Drive TEST", group = "Robot")
public class TELE_RoosterDrive_v1_test extends OpMode {

    //This declares the limelight 3A camera thingy
    Limelight3A limelight;
    // This declares the four motors needed
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;


    Servo FlowerServo;

    Servo GateServo;

    //Shooters + Intake Motor
    DcMotorEx Shooter; //Based of what I can see on cad
    DcMotorEx Transfer; //Based of what I can see on cad
    DcMotor Intake; //Based of what I can see on cad

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    // State Machine Enum for the 4 Subsystem States
    public enum RobotState { //enum is from a website i found, idk if it will work
        IDLE,
        INTAKING,
        SPINNING_UP,
        FEEDING
    }

    private RobotState currentState = RobotState.IDLE;

    // Gate Servo Position Constants -> from website
    public static double GATE_CLOSED_POSITION = 0.0;
    public static double GATE_OPEN_POSITION = 1.0;

    // Target velocity threshold for shooter motor in encoder ticks per second (higher for max speed)
    public static double SHOOTER_TARGET_VELOCITY = 3500.0;

    @Override
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        //added try exceptions because it kept giving me errors because I didn't add one of the servos, so I just added it in case of a problem during the comp.
        try { FlowerServo = hardwareMap.get(Servo.class, "flowerServo"); } catch (Exception e) { FlowerServo = null; }
        try { GateServo = hardwareMap.get(Servo.class, "GateServo"); } catch (Exception e) { try { GateServo = hardwareMap.get(Servo.class, "gateServo"); } catch (Exception e2) { GateServo = null; } }

        try { Shooter = hardwareMap.get(DcMotorEx.class, "shooterMotor"); } catch (Exception e) { try { Shooter = hardwareMap.get(DcMotorEx.class, "BottomShooter"); } catch (Exception e2) { Shooter = null; } }
        try { Transfer = hardwareMap.get(DcMotorEx.class, "transferMotor"); } catch (Exception e) { try { Transfer = hardwareMap.get(DcMotorEx.class, "TopShooter"); } catch (Exception e2) { Transfer = null; } }
        try { Intake = hardwareMap.get(DcMotor.class, "intakeMotor"); } catch (Exception e) { try { Intake = hardwareMap.get(DcMotor.class, "Intake"); } catch (Exception e2) { Intake = null; } }

        // Limelight 3A camera initialization (safely handled if not connected or configured)
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            if (limelight != null) {
                limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
                limelight.pipelineSwitch(0); // Switch to pipeline number 0
                limelight.start(); // This tells Limelight to start looking!
                telemetry.addData("Limelight Status", "Initialized Successfully");
            }
        } catch (Exception e) {
            telemetry.addData("Limelight Warning", "Not found or failed to start: " + e.getMessage());
            limelight = null;
        }

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // Reverse top motor based my convo with ChatGPT


        // This uses RUN_USING_ENCODER to be more accurate. If you don't have the encoder
        // wires, you should remove these lines.
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Shooters + Intake Motor
        if (Shooter != null) Shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (Transfer != null) Transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (Intake != null) Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // IMU Initialization (safely handled if missing or failing)
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                    RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

            RevHubOrientationOnRobot orientationOnRobot = new
                    RevHubOrientationOnRobot(logoDirection, usbDirection);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            telemetry.addData("IMU Status", "Initialized Successfully");
        } catch (Exception e) {
            telemetry.addData("IMU Warning", "Not found or failed to initialize: " + e.getMessage());
            imu = null;
        }

        telemetry.addData("Status", "Initialized Successfully");
    }

    @Override
    public void start() {
        if (FlowerServo != null) {
            FlowerServo.setPosition(0.8);
        }
    }

    @Override
    public void loop() {
        telemetry.addLine("=== CONTROLLER 1 CONTROLS ===");
        telemetry.addLine("Left Stick Y/X: Forward / Strafe");
        telemetry.addLine("Right Stick X : Turn Robot");
        telemetry.addLine("Left Bumper   : Hold for Robot-Relative Drive");
        telemetry.addLine("Right Trigger : Spin-up Shooter -> Auto-Feed");
        telemetry.addLine("Left Trigger  : Intake & Transfer");
        telemetry.addLine("Button A      : Reset Heading / Yaw");
        telemetry.addLine("Dpad Up/Down  : Flower Servo (0.8 / 0.1)");
        telemetry.addLine("=============================");

        // Flower Servo Dpad Controls (Flower servo is only moved by these dpad buttons)
        if (FlowerServo != null) {
            if (gamepad1.dpad_up) {
                FlowerServo.setPosition(0.8);
            } else if (gamepad1.dpad_down) {
                FlowerServo.setPosition(0.1);
            }
        }

        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (gamepad1.a && imu != null) {
            imu.resetYaw();
        }

        // If you press the left bumper or if IMU is not initialized, drive robot relative
        if (gamepad1.left_bumper || imu == null) {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        // Run subsystem state machine periodically
        updateStateMachine();
    }

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    // subsystem thingy
    private void updateStateMachine() {
        boolean rightTriggerPressed = gamepad1.right_trigger > 0.1;
        boolean leftTriggerPressed = gamepad1.left_trigger > 0.1;

        double currentVelocity = (Shooter != null) ? Math.abs(Shooter.getVelocity()) : 0.0;

        switch (currentState) {
            case IDLE:
                setShooterPower(0.0);
                setTransferPower(0.0);
                setIntakePower(0.0);
                setGatePosition(GATE_CLOSED_POSITION);

                if (rightTriggerPressed) {
                    currentState = RobotState.SPINNING_UP;
                } else if (leftTriggerPressed) {
                    currentState = RobotState.INTAKING;
                }
                break;

            case INTAKING:
                setShooterPower(0.0);
                setTransferPower(-1.0);
                setIntakePower(-1.0);
                setGatePosition(GATE_CLOSED_POSITION);

                if (rightTriggerPressed) {
                    currentState = RobotState.SPINNING_UP;
                } else if (!leftTriggerPressed) {
                    currentState = RobotState.IDLE;
                }
                break;

            case SPINNING_UP:
                setShooterPower(-1.0);
                setTransferPower(0.0);
                setIntakePower(0.0);
                setGatePosition(GATE_CLOSED_POSITION);

                if (!rightTriggerPressed) {
                    currentState = RobotState.IDLE;
                } else if (currentVelocity >= SHOOTER_TARGET_VELOCITY) {
                    currentState = RobotState.FEEDING;
                }
                break;

            case FEEDING:
                setShooterPower(-1.0);
                setTransferPower(-1.0);
                setIntakePower(0.0);
                setGatePosition(GATE_OPEN_POSITION);

                if (!rightTriggerPressed) {
                    currentState = RobotState.IDLE;
                }
                break;
        }

        telemetry.addData("Robot State", currentState);
        telemetry.addData("Shooter Velocity (ticks/sec)", "%.2f", currentVelocity);
        telemetry.addData("Target Velocity Threshold", "%.2f", SHOOTER_TARGET_VELOCITY);
    }

    private void setShooterPower(double power) {
        if (Shooter != null) {
            Shooter.setPower(power);
        }
    }

    private void setTransferPower(double power) {
        if (Transfer != null) {
            Transfer.setPower(power);
        }
    }

    private void setIntakePower(double power) {
        if (Intake != null) {
            Intake.setPower(power);
        }
    }

    private void setGatePosition(double position) {
        if (GateServo != null) {
            GateServo.setPosition(position);
        }
    }

    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates (0 deg = Forward)
        double theta = Math.atan2(right, forward);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to Cartesian (cos for forward, sin for right)
        double newForward = r * Math.cos(theta);
        double newRight = r * Math.sin(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }
}