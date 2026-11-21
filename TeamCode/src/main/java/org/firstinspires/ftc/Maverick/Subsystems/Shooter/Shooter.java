package org.firstinspires.ftc.Maverick.Subsystems.Shooter;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.Maverick.Subsystems.Drive.Drive;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {

    public enum ShooterState {
        STOW,
        IDLE_FIXED,
        IDLE_HIVE,
        IDLE_CLOSE,
        SHOOT_HIVE,
        SHOOT_FIXED,
        SHOOT_CLOSE,
    }

    // FALLBACK ("fixed") mode: no odometry, no shot table. The turret drives to one set angle and the
    // flywheel spins to one set speed; the driver aims by pointing the robot, from a known spot.
    public static double FIXED_RPM = 3600.0;      // flywheel speed for the fallback spot (true wheel rpm, old 1700 x 2.25), placeholder: calibrate
    public static double FIXED_TURRET_DEG = 180.0;   // turret angle: 0 = zero stop (shoots out the BACK), 180 = shoots out the FRONT

    // CLOSE mode: same idea as FIXED, for a shot from right up against the HIVE. Low flywheel speed,
    // turret straight out the front; the driver pushes up to the HIVE and points the robot at it.
    public static double CLOSE_RPM = 3200.0;         // placeholder: calibrate at the close-up spot (true wheel rpm)
    public static double CLOSE_TURRET_DEG = 180.0;   // 180 = shoots out the FRONT

    // HOME: when not shooting, the turret parks here so the wiring isn't left stretched out.
    public static double HOME_DEG = 0.0;          // 0 = the zero position it starts in
    public static double HOME_DELAY_S = 0.5;      // wait this long after shooting ends, so a quick re-press doesn't swing it home and back

    // IDLE ROLL: when not shooting, keep the flywheel turning slowly so spin-up starts from here, not 0.
    public static double IDLE_RPM = 1500.0;      // true wheel rpm (old 500 x 2.25); 0 = coast to a stop when idle

    public ShooterState shooterState = ShooterState.STOW;
    public boolean fixedShoot = false;    // right stick press: fallback fixed shot
    public boolean closeShoot = false;    // dpad left: close-up shot (wins over fixedShoot if both are on)

    public double turretBias = 0.0;     // radians, added to the turret setpoint
    public double flywheelBias = 0.0;   // rpm, added to the table rpm
    private double aimCorrectionRad = 0.0;   // from Vision; only used when aiming at the HIVE

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
                turret.getAngleRad(), aimCorrectionRad, flywheelBias);

        switch (shooterState) {
            case STOW:
                azimuthAngle = 0.0;
                flywheelVel = 0.0;
                break;
            case IDLE_HIVE:
            case IDLE_FIXED:
            case IDLE_CLOSE:
                // not shooting: hold where it is for HOME_DELAY_S, then park at home (no live tracking)
                if (goHome) azimuthAngle = Math.toRadians(HOME_DEG);
                // slow roll; off in full-power test mode, where any target would mean full power
                flywheelVel = Flywheel.FULL_POWER_MODE ? 0.0 : IDLE_RPM;
                break;
            case SHOOT_HIVE:
                azimuthAngle = calculatedShot.turretAngleRad;
                flywheelVel = calculatedShot.flywheelRpm;
                break;
            case SHOOT_FIXED:
                azimuthAngle = Math.toRadians(FIXED_TURRET_DEG);
                flywheelVel = FIXED_RPM + flywheelBias;
                break;
            case SHOOT_CLOSE:
                azimuthAngle = Math.toRadians(CLOSE_TURRET_DEG);
                flywheelVel = CLOSE_RPM + flywheelBias;
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
        if (closeShoot) {
            setShooterState(ShooterState.SHOOT_CLOSE);
        } else if (fixedShoot) {
            setShooterState(ShooterState.SHOOT_FIXED);
        } else {
            setShooterState(ShooterState.SHOOT_HIVE);
        }
    }

    public void idleShooter() {
        if (closeShoot) {
            setShooterState(ShooterState.IDLE_CLOSE);
        } else if (fixedShoot) {
            setShooterState(ShooterState.IDLE_FIXED);
        } else {
            setShooterState(ShooterState.IDLE_HIVE);
        }
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
        return shooterState == ShooterState.SHOOT_HIVE || shooterState == ShooterState.SHOOT_FIXED
                || shooterState == ShooterState.SHOOT_CLOSE;
    }

    /** Driver hint only: the flywheel is spinning for a shot and is at speed. Nothing waits on this. */
    public boolean readyToFire() {
        return isShooting() && flywheel.atSpeed();
    }

    /** "HIVE", "FIXED" or "CLOSE", for telemetry. */
    public String modeName() {
        return closeShoot ? "CLOSE" : fixedShoot ? "FIXED" : "HIVE";
    }

    public Turret getTurret() { return turret; }
    public Flywheel getFlywheel() { return flywheel; }
    public ShooterCalculator.ShotSolution getShot() { return calculatedShot; }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter State", shooterState);
        if (closeShoot) telemetry.addData("CLOSE SHOT", "turret %.0f deg, flywheel %.0f rpm, aim with the robot", CLOSE_TURRET_DEG, CLOSE_RPM);
        else if (fixedShoot) telemetry.addData("FALLBACK", "turret %.0f deg, flywheel %.0f rpm, aim with the robot", FIXED_TURRET_DEG, FIXED_RPM);
        telemetry.addData("Distance", "%.1f in", calculatedShot.distanceIn);
        telemetry.addData("Turret", "%.1f / %.1f deg",
                Math.toDegrees(turret.getAngleRad()), Math.toDegrees(azimuthAngle + turretBias));
        telemetry.addData("Flywheel", "%.0f / %.0f rpm", flywheel.getRpm(), flywheel.getTargetRpm());
        telemetry.addData("In table range / at speed", "%b / %b", calculatedShot.valid(), readyToFire());
    }
}