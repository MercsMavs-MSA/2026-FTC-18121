package org.firstinspires.ftc.RoosterDrive_Version1;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    //Shooters + Intake Motor
    DcMotor shooter; //Based of what I can see on cad
    DcMotor transfer; //Based of what I can see on cad
    DcMotor intake; //Based of what I can see on cad

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    // Shooter timer & state tracking for non-blocking spin-up delay
    ElapsedTime shooterTimer = new ElapsedTime();
    boolean wasShooterPressed = false;
    static final double SPIN_UP_TIME_SECONDS = 0.5; // Delay in seconds for shooter flywheel to reach max speed

    @Override
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        flowerServo = hardwareMap.get(Servo.class, "flowerServo"); //DELETE LATER
        //Shooters + Intake Motor

        shooter = hardwareMap.get(DcMotor.class, "shooterMotor");
        transfer = hardwareMap.get(DcMotor.class, "transferMotor");
        intake = hardwareMap.get(DcMotor.class, "intakeMotor");
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
        transfer.setDirection(DcMotor.Direction.REVERSE);

        // This uses RUN_USING_ENCODER to be more accurate. If you don't have the encoder
        // wires, you should remove these lines.
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Shooters + Intake Motor
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        flowerServo.setPosition(0.6);
    }

    @Override
    public void loop() {
        telemetry.addLine("Press A to reset Yaw");
        telemetry.addLine("Hold left bumper to drive in robot relative");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");



        if(gamepad1.dpad_up) {
            flowerServo.setPosition(0.6);
        }

        if(gamepad1.dpad_down) {
            flowerServo.setPosition(0.1);
        }

        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (gamepad1.a) {
            pinpoint.setHeading(0, AngleUnit.RADIANS);
        }

        // If you press the left bumper or if IMU is not initialized, drive robot relative
        if (gamepad1.left_bumper) {
            drive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        } else {
            driveFieldRelative(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        }

        // --- SHOOTER CONTROLS ---
        // Pressing Right Trigger (Controller 1) turns on Shooter motor, waits for spin-up, then turns on transfer motor
//        boolean shootPressed = gamepad1.right_trigger > 0.1;
//        shoot(shootPressed);
//        shooter.setPower(-gamepad1.right_trigger);
        if (gamepad1.right_bumper) {
            shooter.setPower(-1.0);
        } else {
            shooter.setPower(0.0);
        }

        if (gamepad1.y) {
            transfer.setPower(1.0);
        }
//        transfer.setPower(gamepad1.right_trigger);
//        transfer.setPower(gamepad1.left_trigger);
        intake.setPower(-gamepad1.left_trigger);
//        if (gamepad1.x) {
//            intake.setPower(1.0);
////            transfer.setPower(-1.0);
//        }


        // Intake controls: Hold X to intake (1.0), Hold Y to reverse (-1.0)
//        if (intake != null) {
//            if (gamepad1.x) {
//                intake.setPower(1.0);
//            } else if (gamepad1.y) {
//                intake.setPower(-1.0);
//            } else {
//                intake.setPower(0.0);
//            }
//        }

        pinpoint.update();
        Pose2D pose2D = pinpoint.getPosition();

        telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
        telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
        telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));
    }

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    // Shooter function: spins up Shooter flywheel motor first, waits for max speed, then turns on transfer motor
    private void shoot(boolean shootPressed) {
//        if (shootPressed) {
//            if (!wasShooterPressed) {
//                shooterTimer.reset();
//                wasShooterPressed = true;
//            }

            // Step 1: Turn on Shooter motor immediately to spin up flywheel
            if (shooter != null) {
                shooter.setPower(-1.0);
            }

//            // Step 2: Once max spin speed delay is reached, turn on transfer motor to feed
//            if (shooterTimer.seconds() >= SPIN_UP_TIME_SECONDS) {
//                if (transfer != null) {
//                    transfer.setPower(1.0);
//                }
//                telemetry.addData("Shooter Status", "FIRING (transfer Active)");
//            } else {
//                if (transfer != null) {
//                    transfer.setPower(0.0);
//                }
//                telemetry.addData("Shooter Status", "SPINNING UP (%.2f s)", shooterTimer.seconds());
////            }
//        } else {
//            // Trigger released: stop both motors & reset timer state
//            wasShooterPressed = false;
//            if (shooter != null) {
//                shooter.setPower(0.0);
//            }
//            if (transfer != null) {
//                transfer.setPower(0.0);
//            }
//            telemetry.addData("Shooter Status", "OFF");
//        }
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