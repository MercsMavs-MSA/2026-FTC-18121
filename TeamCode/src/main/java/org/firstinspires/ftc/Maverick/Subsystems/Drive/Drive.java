package org.firstinspires.ftc.Maverick.Subsystems.Drive;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Drive {

    public static double MAX_DRIVE_POWER = 0.8;
    public static double SLOW_DRIVE_POWER = 0.3;
    public static double TURN_SCALE = 0.9;
    public static double STICK_EXPONENT = 1;    // 1 = linear, 2 = squared, 3 = cubic

    // Pose saved at the end of auto, used to start teleop where auto left off
    public static Pose2D handoffPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    public static boolean handoffValid = false;   // true once an auto saved a pose; TeleOp uses it once

    private final DcMotor frontLeftDrive;
    private final DcMotor frontRightDrive;
    private final DcMotor backLeftDrive;
    private final DcMotor backRightDrive;
    private final GoBildaPinpointDriver pinpoint;

    private Pose2D pose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    // Field heading pointing straight away from the drivers (Pedro frame: red 0, blue 180 deg)
    private double driverForwardRad = 0.0;

    public Drive(HardwareMap hardwareMap) {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Direct power, not the hub's velocity loop: the velocity loop's soft default tuning ramps the
        // wheels toward the target speed, which makes starting and stopping feel slow and slippery.
        // With BRAKE below, letting go of the stick now stops the robot hard.
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configurePinpoint();
    }


    public void updatePose() {
        pinpoint.update();
        pose = pinpoint.getPosition();
    }

    public void setPose(Pose2D newPose) {
        pinpoint.setPosition(newPose);
        pose = newPose;
    }

    public void setDriverForward(double rad) {
        driverForwardRad = rad;
    }

    /** Press with the robot facing straight away from the drivers. */
    public void resetHeading() {
        pinpoint.setHeading(driverForwardRad, AngleUnit.RADIANS);
    }

    public void saveForTeleop() {
        handoffPose = pose;
        handoffValid = true;
    }

    public Pose2D getPose() { return pose; }
    public double getX() { return pose.getX(DistanceUnit.INCH); }
    public double getY() { return pose.getY(DistanceUnit.INCH); }
    public double getHeadingRad() { return pose.getHeading(AngleUnit.RADIANS); }

    public void teleopDrive(double leftStickY, double leftStickX, double rightStickX,
                            boolean robotRelative, boolean slowMode) {
        double forward = leftStickY; //shape(leftStickY);
        double right = -leftStickX; //shape(-leftStickX);
        double rotate = shape(-rightStickX) * TURN_SCALE;
        double cap = slowMode ? SLOW_DRIVE_POWER : MAX_DRIVE_POWER;

        if (robotRelative) {
            drive(forward, right, rotate, cap);
        } else {
            driveFieldRelative(forward, right, rotate, cap);
        }
    }

    public void stop() {
        drive(0, 0, 0, 0);
    }

    private void driveFieldRelative(double forward, double right, double rotate, double maxSpeed) {
        // First, convert direction being asked to drive to polar coordinates (0 deg = Forward)
        double theta = Math.atan2(right, forward);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta + getHeadingRad() - driverForwardRad);

        // Third, convert back to Cartesian (cos for forward, sin for right)
        double newForward = r * Math.cos(theta);
        double newRight = r * Math.sin(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate, maxSpeed);
    }

    public void drive(double forward, double right, double rotate, double maxSpeed) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }

    // Keeps the sign, bends the response so small stick movements give small powers.
    private static double shape(double x) {
        return Math.signum(x) * Math.pow(Math.abs(x), STICK_EXPONENT);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Pose", "x %.1f in  y %.1f in  h %.1f deg",
                getX(), getY(), pose.getHeading(AngleUnit.DEGREES));
    }

    private void configurePinpoint() {
        // Pod offsets (inches): X pod sideways from center (left +), Y pod forward of center (forward +)
        pinpoint.setOffsets(91.55102, -86.33307, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        // Resets pose to 0,0,0 and recalibrates the IMU: the robot must be still during init
        pinpoint.resetPosAndIMU();
    }
}