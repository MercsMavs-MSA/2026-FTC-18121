package org.firstinspires.ftc.robotcore.external.navigation;

public enum DistanceUnit {
    METER(39.37007874015748), CM(0.3937007874015748), MM(0.03937007874015748), INCH(1.0);

    private final double inchesPerUnit;
    DistanceUnit(double inchesPerUnit) { this.inchesPerUnit = inchesPerUnit; }
    public double toInches(double v) { return v * inchesPerUnit; }
    public double fromInches(double in) { return in / inchesPerUnit; }
}
