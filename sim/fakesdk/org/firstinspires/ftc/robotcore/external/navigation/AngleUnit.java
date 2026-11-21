package org.firstinspires.ftc.robotcore.external.navigation;

public enum AngleUnit {
    DEGREES, RADIANS;

    public static double normalizeRadians(double r) {
        while (r >= Math.PI) r -= 2 * Math.PI;
        while (r < -Math.PI) r += 2 * Math.PI;
        return r;
    }

    public static double normalizeDegrees(double d) {
        while (d >= 180) d -= 360;
        while (d < -180) d += 360;
        return d;
    }

    public double fromRadians(double r) { return this == DEGREES ? Math.toDegrees(r) : r; }
    public double toRadians(double v) { return this == DEGREES ? Math.toRadians(v) : v; }
}
