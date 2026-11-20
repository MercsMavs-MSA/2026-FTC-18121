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

    public ShooterState shooterState = ShooterState.STOW;
    public boolean fixedShoot = false;

    public double turretBias = 0.0;     // radians, added to the turret setpoint
    public double flywheelBias = 0.0;   // rpm, added to the table rpm

    private final Flywheel flywheel;
    private final TurretRTP turret;
    private final Drive drive;

    private double targetX = 0.0;
    private double targetY = 0.0;

    private ShooterCalculator.ShotSolution calculatedShot;
    private double azimuthAngle = 0.0;
    private double flywheelVel = 0.0;

    public Shooter(HardwareMap hardwareMap, Drive drive) {
        flywheel = new Flywheel(hardwareMap);
        turret = new TurretRTP(hardwareMap);
        this.drive = drive;
    }

    public void update() {
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
                azimuthAngle = turret.getAngleRad();
                flywheelVel = 0.0;
                break;
            case IDLE_HIVE:
                azimuthAngle = calculatedShot.turretAngleRad;
                flywheelVel = 0.0;
                break;
            case SHOOT_HIVE:
                azimuthAngle = calculatedShot.turretAngleRad;
                flywheelVel = calculatedShot.flywheelRpm;
                break;
            case SHOOT_FIXED:
                azimuthAngle = turret.getAngleRad();
                flywheelVel = calculatedShot.flywheelRpm;
                break;
            default:
                azimuthAngle = 0.0;
                flywheelVel = 0.0;
                break;
        }

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

    public boolean readyToFire() {
        switch (shooterState) {
            case SHOOT_HIVE:
                return calculatedShot.valid()
                        && ShooterCalculator.isAimed(calculatedShot, turret.getAngleRad() - turretBias)
                        && flywheel.atSpeed();
            case SHOOT_FIXED:
                return flywheel.atSpeed();
            default:
                return false;
        }
    }

    public TurretRTP getTurret() { return turret; }
    public Flywheel getFlywheel() { return flywheel; }
    public ShooterCalculator.ShotSolution getShot() { return calculatedShot; }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter State", shooterState);
        telemetry.addData("Distance", "%.1f in", calculatedShot.distanceIn);
        telemetry.addData("Turret", "%.1f / %.1f deg",
                Math.toDegrees(turret.getAngleRad()), Math.toDegrees(azimuthAngle + turretBias));
        telemetry.addData("Flywheel", "%.0f / %.0f rpm", flywheel.getRpm(), flywheel.getTargetRpm());
        telemetry.addData("Shot valid / ready", "%b / %b", calculatedShot.valid(), readyToFire());
    }
}
