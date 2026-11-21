package sim;

import com.qualcomm.robotcore.hardware.Servo;

/** A simulated servo. It reaches its commanded position after TRAVEL_S seconds. */
public class SimServo implements Servo {
    public final String name;
    public double commanded = Double.NaN;   // NaN = never commanded (unpowered)
    public double physical = Double.NaN;
    public long changedAtNs = 0;
    public boolean commandedBeforeStart = false;
    public int writes = 0;

    public SimServo(String name) { this.name = name; }

    @Override public void setPosition(double p) {
        writes++;
        if (!SimWorld.current.started) commandedBeforeStart = true;
        if (p != commanded) {
            if (Double.isNaN(physical)) physical = p;   // first command: assume it is already roughly there
            commanded = p;
            changedAtNs = SimWorld.nowNanos();
        }
    }
    @Override public double getPosition() { return Double.isNaN(commanded) ? 0 : commanded; }

    void step() {
        if (!Double.isNaN(commanded) && SimWorld.nowNanos() - changedAtNs >= (long) (SimWorld.SERVO_TRAVEL_S * 1e9)) {
            physical = commanded;
        }
    }
}
