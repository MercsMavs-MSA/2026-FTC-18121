package org.firstinspires.ftc.RoosterDrive_Version1;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "18121 Rooster Field Relative Drive", group = "Robot")
public class TELE_RoosterDrive_v1 extends OpMode {

    //This declares the limelight 3A camera thingy
    Limelight3A limelight;

    GoBildaPinpointDriver pinpoint;

    // This declares the four motors needed
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;


    Servo flowerServo;
    Servo GateServo;

    //Shooters + Intake Motor
    DcMotorEx shooter; //Based of what I can see on cad
    DcMotorEx transfer; //Based of what I can see on cad
    DcMotor intake; //Based of what I can see on cad

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    // State Machine Enum for the 4 Subsystem States
    public enum RobotState {
        IDLE,
        INTAKING,
        SPINNING_UP,
        FEEDING
    }

    private RobotState currentState = RobotState.IDLE;

    // Gate Servo Position & Subsystem Power Constants
    public static double GATE_CLOSED_POSITION = 0.0;
    public static double GATE_OPEN_POSITION = 1.0;
    public static double FLOWER_OPEN_POSITION = 0.7;
    public static double FLOWER_CLOSED_POSITION = 0.2;

    // Max power settings for intake, transfer, and shooter motors (100% full speed)
    public static double INTAKE_MAX_POWER = -1.0;
    public static double TRANSFER_MAX_POWER = -1.0;
    public static double SHOOTER_MAX_POWER = -1.0;

    // Live Shooter Power Ratio (starts at 1.0, adjusted by side d-pad in steps of 0.05)
    private double shooterPowerRatio = 1.0;
    private boolean lastDpadRight = false;
    private boolean lastDpadLeft = false;

    private ElapsedTime stateTimer = new ElapsedTime();

    @Override
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        // Hardware map lookups with fallbacks
        try { flowerServo = hardwareMap.get(Servo.class, "flowerServo"); } catch (Exception e1) {
            try { flowerServo = hardwareMap.get(Servo.class, "FlowerServo"); } catch (Exception e2) {
                try { flowerServo = hardwareMap.get(Servo.class, "flower"); } catch (Exception e3) {
                    try { flowerServo = hardwareMap.get(Servo.class, "Flower"); } catch (Exception e4) {
                        flowerServo = null;
                    }
                }
            }
        }
        try { GateServo = hardwareMap.get(Servo.class, "GateServo"); } catch (Exception e1) {
            try { GateServo = hardwareMap.get(Servo.class, "gateServo"); } catch (Exception e2) {
                try { GateServo = hardwareMap.get(Servo.class, "gate"); } catch (Exception e3) {
                    try { GateServo = hardwareMap.get(Servo.class, "Gate"); } catch (Exception e4) {
                        GateServo = null;
                    }
                }
            }
        }

        try { shooter = hardwareMap.get(DcMotorEx.class, "shooterMotor"); } catch (Exception e1) {
            try { shooter = hardwareMap.get(DcMotorEx.class, "BottomShooter"); } catch (Exception e2) {
                try { shooter = hardwareMap.get(DcMotorEx.class, "shooter"); } catch (Exception e3) {
                    shooter = null;
                }
            }
        }
        try { transfer = hardwareMap.get(DcMotorEx.class, "transferMotor"); } catch (Exception e1) {
            try { transfer = hardwareMap.get(DcMotorEx.class, "TopShooter"); } catch (Exception e2) {
                try { transfer = hardwareMap.get(DcMotorEx.class, "transfer"); } catch (Exception e3) {
                    transfer = null;
                }
            }
        }
        try { intake = hardwareMap.get(DcMotor.class, "intakeMotor"); } catch (Exception e1) {
            try { intake = hardwareMap.get(DcMotor.class, "Intake"); } catch (Exception e2) {
                try { intake = hardwareMap.get(DcMotor.class, "intake"); } catch (Exception e3) {
                    intake = null;
                }
            }
        }
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
        if (transfer != null) transfer.setDirection(DcMotor.Direction.REVERSE);

        // This uses RUN_USING_ENCODER to be more accurate. If you don't have the encoder
        // wires, you should remove these lines.
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Shooters + Intake Motor (RUN_WITHOUT_ENCODER sets 100% max speed full raw voltage)
        if (shooter != null) shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (transfer != null) transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (intake != null) intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // IMU Initialization (safely handled if missing or failing)

        // Get a reference to the sensor
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Configure the sensor
        configurePinpoint();

        // Set the location of the robot - this should be the place you are starting the robot from
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        telemetry.addData("Status", "Initialized Successfully");
    }

    @Override
    public void start() {
        if (flowerServo != null) {
            flowerServo.setPosition(FLOWER_OPEN_POSITION);
        }
    }

    @Override
    public void loop() {
        telemetry.addLine("=== CONTROLLER CONTROLS ===");
        telemetry.addLine("Left Stick Y/X: Forward / Strafe");
        telemetry.addLine("Right Stick X : Turn Robot");
        telemetry.addLine("Left Bumper   : Hold for Robot-Relative Drive");
        telemetry.addLine("Right Trigger : Spin-up Shooter (0.3s) -> Shoot & Open Gate");
        telemetry.addLine("Left Trigger  : Intake & Transfer");
        telemetry.addLine("Button A      : Reset Heading / Yaw");
        telemetry.addLine("Dpad Up       : Flower Servo Open (0.7)");
        telemetry.addLine("Dpad Down     : Flower Servo Closed (0.2)");
        telemetry.addLine("Dpad Right    : Increase Shooter Speed (+0.05)");
        telemetry.addLine("Dpad Left     : Decrease Shooter Speed (-0.05)");
        telemetry.addLine("=============================");

        // Flower Servo Dpad Controls (supports Gamepad 1 & 2)
        boolean dpadUp = gamepad1.dpad_up || gamepad2.dpad_up;
        boolean dpadDown = gamepad1.dpad_down || gamepad2.dpad_down;

        if (flowerServo != null) {
            if (dpadUp) {
                flowerServo.setPosition(FLOWER_OPEN_POSITION);
            } else if (dpadDown) {
                flowerServo.setPosition(FLOWER_CLOSED_POSITION);
            }
        }

        // Side D-pad Controls (Dpad Left/Right) to adjust shooter power ratio live (starts at 1.0, steps of 0.05)
        boolean dpadRight = gamepad1.dpad_right || gamepad2.dpad_right;
        boolean dpadLeft = gamepad1.dpad_left || gamepad2.dpad_left;

        if (dpadRight && !lastDpadRight) {
            shooterPowerRatio = Math.min(1.0, shooterPowerRatio + 0.01);
        }
        if (dpadLeft && !lastDpadLeft) {
            shooterPowerRatio = Math.max(0.0, shooterPowerRatio - 0.01);
        }
        lastDpadRight = dpadRight;
        lastDpadLeft = dpadLeft;

        // Reset Pinpoint Odometry Heading on button A
        if (gamepad1.a && pinpoint != null) {
            pinpoint.setHeading(0, AngleUnit.RADIANS);
        }

        // Drive logic (Field-relative or Robot-relative)
        if (gamepad1.left_bumper || pinpoint == null) {
            drive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        } else {
            driveFieldRelative(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        }

        // Run subsystem State Machine periodically in loop
        updateStateMachine();

        if (pinpoint != null) {
            pinpoint.update();
            Pose2D pose2D = pinpoint.getPosition();
            telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
            telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
            telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));
        }
    }

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    // --- SUBSYSTEM STATE MACHINE ---
    private void updateStateMachine() {
        boolean rightTriggerPressed = gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1;
        boolean leftTriggerPressed = gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1;

        double currentVelocity = (shooter != null) ? Math.abs(shooter.getVelocity()) : 0.0;

        switch (currentState) {
            case IDLE:
                // IDLE: default state - all motors off, gate closed
                setShooterPower(0.0);
                setTransferPower(0.0);
                setIntakePower(0.0);
                setGatePosition(GATE_CLOSED_POSITION);

                if (rightTriggerPressed) {
                    currentState = RobotState.SPINNING_UP;
                    stateTimer.reset();
                } else if (leftTriggerPressed) {
                    currentState = RobotState.INTAKING;
                }
                break;

            case INTAKING:
                // INTAKING: run both intake and transfer at max speed, gate closed
                setShooterPower(0.0);
                setTransferPower(TRANSFER_MAX_POWER);
                setIntakePower(INTAKE_MAX_POWER);
                setGatePosition(GATE_CLOSED_POSITION);

                if (rightTriggerPressed) {
                    currentState = RobotState.SPINNING_UP;
                    stateTimer.reset();
                } else if (!leftTriggerPressed) {
                    currentState = RobotState.IDLE;
                }
                break;

            case SPINNING_UP:
                // SPINNING_UP (0 to 0.3s): shooter spins up at live RPM, transfer off, gate closed
                setShooterPower(SHOOTER_MAX_POWER * shooterPowerRatio);
                setTransferPower(0.0);
                setIntakePower(0.0);
                setGatePosition(GATE_CLOSED_POSITION);

                if (!rightTriggerPressed) {
                    currentState = RobotState.IDLE;
                } else if (stateTimer.seconds() >= 0.3) {
                    currentState = RobotState.FEEDING;
                }
                break;

            case FEEDING:
                // FEEDING / SHOOTING (after 0.3s): shooter runs, transfer & intake run, gate OPEN
                setShooterPower(SHOOTER_MAX_POWER * shooterPowerRatio);
                setTransferPower(TRANSFER_MAX_POWER);
                setIntakePower(INTAKE_MAX_POWER);
                setGatePosition(GATE_OPEN_POSITION);

                if (!rightTriggerPressed) {
                    currentState = RobotState.IDLE;
                }
                break;
        }

        telemetry.addData("Flower Servo Status", flowerServo != null ? String.format(java.util.Locale.US, "Connected (Pos: %.2f)", flowerServo.getPosition()) : "NOT FOUND (Check Robot Config)");
        telemetry.addData("Gate Servo Status", GateServo != null ? String.format(java.util.Locale.US, "Connected (Pos: %.2f)", GateServo.getPosition()) : "NOT FOUND (Check Robot Config)");
        telemetry.addData("Robot State", currentState);
        telemetry.addData("Shooter Power Ratio", "%.2f (%.0f%%)", shooterPowerRatio, shooterPowerRatio * 100);
        telemetry.addData("Shooter Velocity (ticks/sec)", "%.2f", currentVelocity);
    }

    private void setShooterPower(double power) {
        if (shooter != null) {
            shooter.setPower(power);
        }
    }

    private void setTransferPower(double power) {
        if (transfer != null) {
            transfer.setPower(power);
        }
    }

    private void setIntakePower(double power) {
        if (intake != null) {
            intake.setPower(power);
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
        theta = AngleUnit.normalizeRadians(theta +
                pinpoint.getHeading(AngleUnit.RADIANS));

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

    public void configurePinpoint(){
        /*
         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
         *
         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         *  Left of the center is a positive number, right of center is a negative number.
         *
         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         */
        pinpoint.setOffsets(-7.26, 0, DistanceUnit.INCH); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
         * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
         * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
         * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
         * number of ticks per unit of your odometry pod.  For example:
         *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
         */
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        pinpoint.resetPosAndIMU();
    }
}