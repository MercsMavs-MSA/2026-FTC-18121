package org.firstinspires.ftc.Maverick.Subsystems.Shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Turret {

    // 200/48 ratio
    // CUSTOM PID VERSION

    public static double TICKS_PER_TURRET_REV = 103.8 * 200.0 / 48.0;
    public static double kP = 1.5;   // power per radian of error, placeholder
    public static double kD = 0.05;  // power per rad/s of turret speed, placeholder
    public static double kS = 0.04;  // minimum power to overcome friction, placeholder
    public static double MAX_POWER = 0.8;
    public static double DEADBAND_DEG = 0.5;
    public static double POWER_DEADBAND = 0.01;

    public static double handoffAngleRad = 0.0; // turret angle saved at the end of auto, read by teleop

    private final DcMotorEx turretMotor;

    private double offsetTicks = 0;
    private double targetRad = 0;
    private double angleRad = 0;
    private double velocityRadPerSec = 0;
    private double lastPower = Double.NaN;
    private boolean enabled = false;
    private boolean atSoftLimit = false;

    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setPower(0);
    }

    public void zeroAt(double currentAngleRad) {
        offsetTicks = currentAngleRad / (2.0 * Math.PI) * TICKS_PER_TURRET_REV - turretMotor.getCurrentPosition();
        angleRad = currentAngleRad;
        targetRad = currentAngleRad;
    }

    public void setTargetAngle(double rad) {
        targetRad = Range.clip(rad, softMinRad(), softMaxRad());
        enabled = true;
    }

    public void stop() {
        enabled = false;
        writePower(0);
    }

    public void update() {
        double ticks = turretMotor.getCurrentPosition() + offsetTicks;
        angleRad = ticks / TICKS_PER_TURRET_REV * 2.0 * Math.PI;
        velocityRadPerSec = turretMotor.getVelocity() / TICKS_PER_TURRET_REV * 2.0 * Math.PI;

        if (!enabled) {
            writePower(0);
            return;
        }

        double error = targetRad - angleRad;
        double power = 0;
        if (Math.abs(error) > Math.toRadians(DEADBAND_DEG)) {
            power = kP * error - kD * velocityRadPerSec + kS * Math.signum(error);
        }
        power = Range.clip(power, -MAX_POWER, MAX_POWER);

        atSoftLimit = false;
        if (angleRad >= softMaxRad() && power > 0) {
            power = 0;
            atSoftLimit = true;
        }
        if (angleRad <= softMinRad() && power < 0) {
            power = 0;
            atSoftLimit = true;
        }

        writePower(power);
    }

    private void writePower(double power) {
        if (Double.isNaN(lastPower) || Math.abs(power - lastPower) >= POWER_DEADBAND || (power == 0 && lastPower != 0)) {
            turretMotor.setPower(power);
            lastPower = power;
        }
    }

    private static double softMinRad() {
        return ShooterCalculator.TURRET_MIN_RAD;
    }

    private static double softMaxRad() {
        return ShooterCalculator.TURRET_MAX_RAD;
    }

    public void saveForTeleop() {
        handoffAngleRad = angleRad;
    }

    public boolean atTarget(double toleranceRad) {
        return enabled && Math.abs(targetRad - angleRad) <= toleranceRad;
    }

    public double getAngleRad()         { return angleRad; }
    public double getTargetRad()        { return targetRad; }
    public double getVelocityRadPerSec(){ return velocityRadPerSec; }
    public boolean isAtSoftLimit()      { return atSoftLimit; }
}