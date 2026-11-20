package org.firstinspires.ftc.Maverick.Subsystems.Shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class TurretRTP {

    //RUN TO POSITION VERSION USING ONLY P, MOST LIKELY LESS PRECISE

    public static double TICKS_PER_TURRET_REV = 103.8 * 200.0 / 48.0;
    public static double MAX_POWER = 0.8;         // speed cap in RUN_TO_POSITION
    public static double TOLERANCE_DEG = 1.0;

    public static double handoffAngleRad = 0.0;

    private final DcMotorEx turretMotor;
    private double offsetTicks = 0;
    private double targetRad = 0;
    private double angleRad = 0;
    private int sentTicks = Integer.MIN_VALUE;

    public TurretRTP(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0);
    }

    public void setTargetAngle(double rad) {
        targetRad = Range.clip(rad, ShooterCalculator.TURRET_MIN_RAD, ShooterCalculator.TURRET_MAX_RAD);
    }

    public void update() {
        angleRad = (turretMotor.getCurrentPosition() + offsetTicks) / TICKS_PER_TURRET_REV * 2 * Math.PI;

        int ticks = (int) Math.round(targetRad / (2 * Math.PI) * TICKS_PER_TURRET_REV - offsetTicks);
        if (ticks != sentTicks) {
            turretMotor.setTargetPosition(ticks);
            turretMotor.setPower(MAX_POWER);
            sentTicks = ticks;
        }
    }

    public void stop() {
        turretMotor.setPower(0);
        sentTicks = Integer.MIN_VALUE;
    }

    public void zeroAt(double currentAngleRad) {
        offsetTicks = currentAngleRad / (2 * Math.PI) * TICKS_PER_TURRET_REV - turretMotor.getCurrentPosition();
        sentTicks = Integer.MIN_VALUE;
    }

    public void saveForTeleop() { handoffAngleRad = angleRad; }

    public boolean atTarget() { return Math.abs(targetRad - angleRad) <= Math.toRadians(TOLERANCE_DEG); }
    public double getAngleRad() { return angleRad; }
    public double getTargetRad() { return targetRad; }
}