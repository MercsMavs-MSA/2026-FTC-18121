package org.firstinspires.ftc.robotcore.external;

public interface Telemetry {
    Object addData(String caption, Object value);
    Object addData(String caption, String format, Object... args);
    Object addLine(String line);
    Object addLine();
    boolean update();
}
