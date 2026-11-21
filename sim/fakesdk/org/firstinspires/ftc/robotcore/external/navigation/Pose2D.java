package org.firstinspires.ftc.robotcore.external.navigation;

public class Pose2D {
    private final double xIn, yIn, headingRad;

    public Pose2D(DistanceUnit distanceUnit, double x, double y, AngleUnit angleUnit, double heading) {
        xIn = distanceUnit.toInches(x);
        yIn = distanceUnit.toInches(y);
        headingRad = angleUnit.toRadians(heading);
    }

    public double getX(DistanceUnit unit) { return unit.fromInches(xIn); }
    public double getY(DistanceUnit unit) { return unit.fromInches(yIn); }
    public double getHeading(AngleUnit unit) { return unit.fromRadians(headingRad); }
}
