package sim;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;

/** Captures what the Driver Station would show. lastFrame holds the most recent telemetry.update(). */
public class SimTelemetry implements Telemetry {
    private Map<String, String> building = new LinkedHashMap<>();
    public Map<String, String> lastFrame = new LinkedHashMap<>();
    private int lines = 0;

    @Override public Object addData(String caption, Object value) { building.put(caption, String.valueOf(value)); return null; }
    @Override public Object addData(String caption, String format, Object... args) {
        building.put(caption, String.format(Locale.US, format, args));
        return null;
    }
    @Override public Object addLine(String line) { building.put("line" + (lines++), line); return null; }
    @Override public Object addLine() { return addLine(""); }
    @Override public boolean update() {
        lastFrame = building;
        building = new LinkedHashMap<>();
        lines = 0;
        return true;
    }

    public String get(String caption) { return lastFrame.getOrDefault(caption, ""); }
}
