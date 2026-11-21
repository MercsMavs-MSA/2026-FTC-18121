package org.firstinspires.ftc.Maverick.Subsystems.Shooter;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.Maverick.Subsystems.Drive.Drive;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {

    public enum ShooterState {
        STOW,
        IDLE_FIXED,
        IDLE_HIVE,
        SHOOT_HIVE,
        SHOOT_FIXED,
    }

    public static double FIXED_RPM = 2400.0;   // constant flywheel speed in fixed mode, placeholder

    public ShooterState shooterState = ShooterState.STOW;
    public boolean fixedShoot = false;

    public double turretBias = 0.0;     // radians, added to the turret setpoint
    public double flywheelBias = 0.0;   // rpm, added to the table rpm

    private final Flywheel flywheel;
    private final Turret turret;
    private final Drive drive;

    private double targetX = 0.0;
    private double targetY = 0.0;

    private ShooterCalculator.ShotSolution calculatedShot;
    private double azimuthAngle = 0.0;
    private double flywheelVel = 0.0;
    private double heldAngle = 0.0;        // where the turret stays when it is not aiming
    private boolean heldAngleSet = false;

    public Shooter(HardwareMap hardwareMap, Drive drive) {
        flywheel = new Flywheel(hardwareMap);
        turret = new Turret(hardwareMap);
        this.drive = drive;
    }

    public void update() {
        if (!heldAngleSet) {
            heldAngle = turret.getAngleRad();   // first loop: hold wherever the turret started
            heldAngleSet = true;
        }

        calculatedShot = ShooterCalculator.solve(
                drive.getX(), drive.getY(), drive.getHeadingRad(),
                targetX, targetY,
                turret.getAngleRad(), 0.0, flywheelBias);

        switch (shooterState) {
            case STOW:
                azimuthAngle = 0.0;
                flywheelVel = 0.0;
                break;
            case IDLE_FIXED:
            case IDLE_HIVE:
                azimuthAngle = heldAngle;           // no live tracking: stay put until the trigger
                flywheelVel = 0.0;
                break;
            case SHOOT_HIVE:
                azimuthAngle = calculatedShot.turretAngleRad;
                flywheelVel = calculatedShot.flywheelRpm;
                break;
            case SHOOT_FIXED:
                azimuthAngle = heldAngle;           // locked where it is
                flywheelVel = FIXED_RPM + flywheelBias;
                break;
            default:
                azimuthAngle = 0.0;
                flywheelVel = 0.0;
                break;
        }
        heldAngle = azimuthAngle;

        turret.setTargetAngle(azimuthAngle + turretBias);
        flywheel.setTargetRpm(flywheelVel);

        turret.update();
        flywheel.update();
    }

    public void setShooterState(ShooterState state) {
        shooterState = state;
    }

    public void startShooter() {
        if (fixedShoot) {
            setShooterState(ShooterState.SHOOT_FIXED);
        } else {
            setShooterState(ShooterState.SHOOT_HIVE);
        }
    }

    public void idleShooter() {
        if (fixedShoot) {
            setShooterState(ShooterState.IDLE_FIXED);
        } else {
            setShooterState(ShooterState.IDLE_HIVE);
        }
    }

    public void setTarget(double x, double y) {
        targetX = x;
        targetY = y;
    }

    public void stop() {
        shooterState = ShooterState.STOW;
        flywheel.stop();
    }

    public boolean isFlywheelAtThreshold() {
        return flywheel.atSpeed();
    }

    /** Driver hint only: the flywheel is spinning for a shot and is at speed. Nothing waits on this. */
    public boolean readyToFire() {
        boolean shooting = shooterState == ShooterState.SHOOT_HIVE || shooterState == ShooterState.SHOOT_FIXED;
        return shooting && flywheel.atSpeed();
    }

    public Turret getTurret() { return turret; }
    public Flywheel getFlywheel() { return flywheel; }
    public ShooterCalculator.ShotSolution getShot() { return calculatedShot; }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter State", shooterState);
        telemetry.addData("Distance", "%.1f in", calculatedShot.distanceIn);
        telemetry.addData("Turret", "%.1f / %.1f deg",
                Math.toDegrees(turret.getAngleRad()), Math.toDegrees(azimuthAngle + turretBias));
        telemetry.addData("Flywheel", "%.0f / %.0f rpm", flywheel.getRpm(), flywheel.getTargetRpm());
        telemetry.addData("In table range / at speed", "%b / %b", calculatedShot.valid(), readyToFire());
    }
}