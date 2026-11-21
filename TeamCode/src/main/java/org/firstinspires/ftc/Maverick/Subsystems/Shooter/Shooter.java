package org.firstinspires.ftc.Maverick.Subsystems.Shooter;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.Maverick.Subsystems.Drive.Drive;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Turret aims at the HIVE (odometry + vision correction). The flywheel uses one of two constant speeds:
 * CLOSE_RPM (default) or FAR_RPM. Back toggles between them (Robot.java).
 */
public class Shooter {

    public enum ShooterState {
        STOW,
        IDLE,     // not shooting: flywheel at the idle roll, turret parks home (or keeps aiming, see TRACK_WHEN_IDLE)
        SHOOT,    // RT / A held: turret aims, flywheel at CLOSE_RPM or FAR_RPM
    }

    // ===== The two shot speeds (true wheel rpm) =====
    public static double CLOSE_RPM = 3250.0;   // default at the start of every OpMode
    public static double FAR_RPM = 3600.0;

    // ===== Turret =====
    // false: the turret aims only while shooting and parks at HOME_DEG otherwise (keeps the wiring slack).
    // true:  the turret keeps aiming at the HIVE all the time.
    public static boolean TRACK_WHEN_IDLE = false;
    public static double HOME_DEG = 0.0;          // 0 = the zero position it starts in
    public static double HOME_DELAY_S = 0.5;      // wait this long after shooting ends, so a quick re-press doesn't swing it home and back

    // IDLE ROLL: when not shooting, keep the flywheel turning slowly so spin-up starts from here, not 0.
    public static double IDLE_RPM = 1500.0;      // true wheel rpm; 0 = coast to a stop when idle

    public ShooterState shooterState = ShooterState.STOW;
    public boolean farShot = false;     // false = CLOSE_RPM (default), true = FAR_RPM

    public double turretBias = 0.0;     // radians, added to the turret setpoint
    public double flywheelBias = 0.0;   // rpm, added to the shot speed
    private double aimCorrectionRad = 0.0;   // from Vision

    private final Flywheel flywheel;
    private final Turret turret;
    private final Drive drive;

    private double targetX = 0.0;
    private double targetY = 0.0;

    private ShooterCalculator.ShotSolution calculatedShot;
    private double azimuthAngle = 0.0;
    private double flywheelVel = 0.0;
    private final ElapsedTime sinceShooting = new ElapsedTime();   // time since the shooter last aimed for a shot

    public Shooter(HardwareMap hardwareMap, Drive drive) {
        flywheel = new Flywheel(hardwareMap);
        turret = new Turret(hardwareMap);
        this.drive = drive;
    }

    public void update() {
        if (isShooting()) sinceShooting.reset();
        boolean goHome = sinceShooting.seconds() >= HOME_DELAY_S;

        calculatedShot = ShooterCalculator.solve(
                drive.getX(), drive.getY(), drive.getHeadingRad(),
                targetX, targetY,
                turret.getAngleRad(), aimCorrectionRad);

        switch (shooterState) {
            case SHOOT:
                azimuthAngle = calculatedShot.turretAngleRad;
                flywheelVel = shotRpm() + flywheelBias;
                break;
            case IDLE:
                if (TRACK_WHEN_IDLE) {
                    azimuthAngle = calculatedShot.turretAngleRad;
                } else if (goHome) {
                    azimuthAngle = Math.toRadians(HOME_DEG);   // hold where it is for HOME_DELAY_S, then park
                }
                // slow roll; off in full-power test mode, where any target would mean full power
                flywheelVel = Flywheel.FULL_POWER_MODE ? 0.0 : IDLE_RPM;
                break;
            case STOW:
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

    /** The selected shot speed: CLOSE_RPM or FAR_RPM. */
    public double shotRpm() {
        return farShot ? FAR_RPM : CLOSE_RPM;
    }

    public void toggleShotSpeed() {
        farShot = !farShot;
    }

    public void setShooterState(ShooterState state) {
        shooterState = state;
    }

    public void startShooter() {
        setShooterState(ShooterState.SHOOT);
    }

    public void idleShooter() {
        setShooterState(ShooterState.IDLE);
    }

    /** Vision's aim correction in radians (+ = aim further left / CCW). Pass 0 for odometry only. */
    public void setAimCorrection(double rad) {
        aimCorrectionRad = rad;
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

    public boolean isShooting() {
        return shooterState == ShooterState.SHOOT;
    }

    /** Driver hint only: the flywheel is spinning for a shot and is at speed. Nothing waits on this. */
    public boolean readyToFire() {
        return isShooting() && flywheel.atSpeed();
    }

    /** "CLOSE" or "FAR", for telemetry. */
    public String shotName() {
        return farShot ? "FAR" : "CLOSE";
    }

    public Turret getTurret() { return turret; }
    public Flywheel getFlywheel() { return flywheel; }
    public ShooterCalculator.ShotSolution getShot() { return calculatedShot; }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter State", shooterState);
        telemetry.addData("Shot speed", "%s  %.0f rpm   (Back toggles)", shotName(), shotRpm());
        telemetry.addData("Distance", "%.1f in", calculatedShot.distanceIn);
        telemetry.addData("Turret", "%.1f / %.1f deg%s",
                Math.toDegrees(turret.getAngleRad()), Math.toDegrees(azimuthAngle + turretBias),
                calculatedShot.reachable ? "" : " (target outside turret travel)");
        telemetry.addData("Flywheel", "%.0f / %.0f rpm", flywheel.getRpm(), flywheel.getTargetRpm());
        telemetry.addData("At speed", readyToFire());
    }
}