package org.firstinspires.ftc.Maverick.Subsystems.Shooter;

/**
 * Turret aiming only: where the turret must point to hit the target, from the robot pose.
 * (No flywheel speed: Shooter uses two constant speeds, CLOSE_RPM and FAR_RPM.)
 */
public class ShooterCalculator {
    public static final double TURRET_OFFSET_FORWARD_IN = -2.71;
    public static final double TURRET_OFFSET_LEFT_IN = 0.0;

    public static final double TURRET_MIN_DEG = 0.0;
    public static final double TURRET_MAX_DEG = 360.0;
    public static final double TURRET_ZERO_OFFSET_DEG = 180.0;

    public static final double TURRET_MIN_RAD = Math.toRadians(TURRET_MIN_DEG);
    public static final double TURRET_MAX_RAD = Math.toRadians(TURRET_MAX_DEG);
    public static final double TURRET_ZERO_OFFSET_RAD = Math.toRadians(TURRET_ZERO_OFFSET_DEG);
    public static final double LIMIT_TOLERANCE_RAD = Math.toRadians(2.0);

    public static final double TARGET_HALF_WIDTH_IN = 10.0;
    public static final double BALL_RADIUS_IN = 1.4;

    public ShooterCalculator() {}

    public static ShotSolution solve(double robotX, double robotY, double robotHeadingRad, double targetX, double targetY,
                                     double turretAngleNowRad, double aimTrimRad) {
        return compute(robotX, robotY, robotHeadingRad, targetX, targetY, turretAngleNowRad, aimTrimRad,
                TURRET_OFFSET_FORWARD_IN, TURRET_OFFSET_LEFT_IN, TURRET_MIN_RAD, TURRET_MAX_RAD,
                TURRET_ZERO_OFFSET_RAD, TARGET_HALF_WIDTH_IN, BALL_RADIUS_IN);
    }

    /** Old signature (with a rpm trim), kept so older OpModes such as ShotCalibration still compile. The rpm trim is ignored. */
    public static ShotSolution solve(double robotX, double robotY, double robotHeadingRad, double targetX, double targetY,
                                     double turretAngleNowRad, double aimTrimRad, double rpmTrimIgnored) {
        return solve(robotX, robotY, robotHeadingRad, targetX, targetY, turretAngleNowRad, aimTrimRad);
    }

    static ShotSolution compute(double robotX, double robotY, double robotHeadingRad, double targetX, double targetY, double turretAngleNowRad, double aimTrimRad, double offForwardIn, double offLeftIn, double minRad, double maxRad, double zeroOffsetRad, double halfWidthIn, double ballRadiusIn) {
        // Turret pivot in the field frame: rotate the robot-frame offset by the heading.
        double cos = Math.cos(robotHeadingRad);
        double sin = Math.sin(robotHeadingRad);
        double turretX = robotX + offForwardIn * cos - offLeftIn * sin;
        double turretY = robotY + offForwardIn * sin + offLeftIn * cos;

        double dx = targetX - turretX;
        double dy = targetY - turretY;
        double distance = Math.hypot(dx, dy);
        double fieldAngle = Math.atan2(dy, dx);

        // Where the turret should point, relative to the robot, within one turn.
        double desired = wrap(fieldAngle - robotHeadingRad - zeroOffsetRad + aimTrimRad);

        double best = Double.NaN;
        for (int k = -1; k <= 2; k++) {
            double c = desired + k * 2.0 * Math.PI;
            if (c >= minRad - LIMIT_TOLERANCE_RAD && c <= maxRad + LIMIT_TOLERANCE_RAD && (Double.isNaN(best) || Math.abs(c - turretAngleNowRad) < Math.abs(best - turretAngleNowRad))) {
                best = c;
            }
        }
        boolean reachable = !Double.isNaN(best);
        if (!reachable) best = desired;
        best = Math.max(minRad, Math.min(maxRad, best));

        // How far off the aim can be and still go in: the opening's half width minus the ball radius.
        double tolerance = Math.atan2(Math.max(0.0, halfWidthIn - ballRadiusIn), Math.max(distance, 1.0));

        return new ShotSolution(best, fieldAngle, distance, tolerance, reachable);
    }

    static double wrap(double a) {
        return a - 2.0 * Math.PI * Math.floor((a + Math.PI) / (2.0 * Math.PI));
    }

    public static boolean isAimed(ShotSolution s, double turretAngleNowRad) {
        return s.reachable && Math.abs(turretAngleNowRad - s.turretAngleRad) <= s.aimToleranceRad;
    }

    public static final class ShotSolution {
        public final double turretAngleRad;
        public final double fieldAngleRad;
        public final double distanceIn;
        public final double aimToleranceRad;
        public final boolean reachable;

        ShotSolution(double turretAngleRad, double fieldAngleRad, double distanceIn,
                     double aimToleranceRad, boolean reachable) {
            this.turretAngleRad = turretAngleRad;
            this.fieldAngleRad = fieldAngleRad;
            this.distanceIn = distanceIn;
            this.aimToleranceRad = aimToleranceRad;
            this.reachable = reachable;
        }
        public boolean valid() {
            return reachable;
        }
    }
}