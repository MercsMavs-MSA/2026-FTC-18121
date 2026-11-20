package org.firstinspires.ftc.Maverick.Subsystems.Shooter;

import java.util.ArrayList;
import java.util.List;
public class ShooterCalculator { //TODO: after drivebase code is created, wrte an opmode to gather successful shotmap values from different areas on the field.
    public static final double TURRET_OFFSET_FORWARD_IN = 0.0;
    public static final double TURRET_OFFSET_LEFT_IN = 0.0;

    public static final double TURRET_MIN_DEG = -180.0;
    public static final double TURRET_MAX_DEG = 180.0;
    public static final double TURRET_ZERO_OFFSET_DEG = 0.0;

    public static final double TURRET_MIN_RAD = Math.toRadians(TURRET_MIN_DEG);
    public static final double TURRET_MAX_RAD = Math.toRadians(TURRET_MAX_DEG);
    public static final double TURRET_ZERO_OFFSET_RAD = Math.toRadians(TURRET_ZERO_OFFSET_DEG);

    public static final double TARGET_HALF_WIDTH_IN = 10.0;
    public static final double BALL_RADIUS_IN = 1.4;

    public double ballRadiusIn = 1.4;

    public static final double[][] SHOT_TABLE = { //placeholder/temporary values
            {  48, 2025 },
            {  54, 2075 },
            {  60, 2125 },
            {  66, 2175 },
            {  72, 2225 },
            {  78, 2275 },
            {  84, 2325 },
            {  90, 2400 },
            {  96, 2450 },
            { 102, 2500 },
            { 108, 2550 },
            { 114, 2625 },
            { 120, 2675 },
            { 126, 2725 },
            { 132, 2775 },
    };
    public ShooterCalculator() {}

    public static ShotSolution solve(double robotX, double robotY, double robotHeadingRad, double targetX, double targetY, double turretAngleNowRad, double aimTrimRad, double rpmTrim) {
        return compute(robotX, robotY, robotHeadingRad, targetX, targetY, turretAngleNowRad, aimTrimRad, rpmTrim,
                TURRET_OFFSET_FORWARD_IN, TURRET_OFFSET_LEFT_IN, TURRET_MIN_RAD, TURRET_MAX_RAD,
                TURRET_ZERO_OFFSET_RAD, TARGET_HALF_WIDTH_IN, BALL_RADIUS_IN, SHOT_TABLE);
    }

    static ShotSolution compute(double robotX, double robotY, double robotHeadingRad, double targetX, double targetY, double turretAngleNowRad, double aimTrimRad, double rpmTrim, double offForwardIn, double offLeftIn, double minRad, double maxRad, double zeroOffsetRad, double halfWidthIn, double ballRadiusIn, double[][] table) {
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

        if (turretAngleNowRad > 0 && desired + 2.0 * Math.PI <= maxRad) desired += 2.0 * Math.PI;
        if (turretAngleNowRad < 0 && desired - 2.0 * Math.PI >= minRad) desired -= 2.0 * Math.PI;
        boolean reachable = desired >= minRad && desired <= maxRad;
        double best = Math.max(minRad, Math.min(maxRad, desired));

        // Flywheel speed from the table, plus the driver trim.
        boolean tableOk = table.length >= 2;
        boolean inTableRange = tableOk && distance >= table[0][0] && distance <= table[table.length - 1][0];
        double rpm = tableOk ? Math.max(0.0, rpmAt(table, distance) + rpmTrim) : 0.0;

        // How far off the aim can be and still go in: the opening's half width minus the ball radius.
        double tolerance = Math.atan2(Math.max(0.0, halfWidthIn - ballRadiusIn), Math.max(distance, 1.0));

        return new ShotSolution(best, fieldAngle, distance, rpm, tolerance, reachable, inTableRange);
    }

    /** Shotmap interpolation. */
    static double rpmAt(double[][] table, double d) {
        if (table.length == 0) return 0.0;
        if (d <= table[0][0]) return table[0][1];
        double[] last = table[table.length - 1];
        if (d >= last[0]) return last[1];
        for (int i = 0; i < table.length - 1; i++) {
            double[] a = table[i];
            double[] b = table[i + 1];
            if (d >= a[0] && d <= b[0]) {
                return a[1] + (d - a[0]) / (b[0] - a[0]) * (b[1] - a[1]);
            }
        }
        return last[1];
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
        public final double flywheelRpm;
        public final double aimToleranceRad;
        public final boolean reachable;
        public final boolean inTableRange;

        ShotSolution(double turretAngleRad, double fieldAngleRad, double distanceIn, double flywheelRpm,
                     double aimToleranceRad, boolean reachable, boolean inTableRange) {
            this.turretAngleRad = turretAngleRad;
            this.fieldAngleRad = fieldAngleRad;
            this.distanceIn = distanceIn;
            this.flywheelRpm = flywheelRpm;
            this.aimToleranceRad = aimToleranceRad;
            this.reachable = reachable;
            this.inTableRange = inTableRange;
        }
        public boolean valid() {
            return reachable && inTableRange;
        }
    }

}
