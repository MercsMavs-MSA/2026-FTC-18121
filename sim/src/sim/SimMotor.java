package sim;

import com.qualcomm.robotcore.hardware.DcMotorEx;

/** A simulated motor + encoder. SimWorld moves it each step from the commands it has received. */
public class SimMotor implements DcMotorEx {
    public final String name;
    public final double freeTicksPerSec;   // encoder ticks/s at full power, no load

    public Direction direction = Direction.FORWARD;
    public RunMode mode = RunMode.RUN_WITHOUT_ENCODER;
    public ZeroPowerBehavior zeroPower = ZeroPowerBehavior.BRAKE;   // SDK default for most motors
    public double power = 0;                 // last setPower, as commanded
    public double velocityTarget = 0;        // ticks/s, RUN_USING_ENCODER
    public int targetPosition = 0;

    public double position = 0;              // true encoder position, ticks (signed in FORWARD sense)
    public double velocity = 0;              // true ticks/s
    public double encoderZero = 0;
    public final double[] velHistory = new double[3];   // hub velocity reading is a short moving average

    public int writes = 0;
    public boolean commandedBeforeStart = false;
    public boolean pidfSet = false;

    public SimMotor(String name, double freeTicksPerSec) {
        this.name = name;
        this.freeTicksPerSec = freeTicksPerSec;
    }

    private double sign() { return direction == Direction.REVERSE ? -1 : 1; }

    /** Power after the SDK's direction flip, i.e. what the motor actually sees. */
    public double appliedPower() { return power * sign(); }

    @Override public void setDirection(Direction d) { direction = d; }
    @Override public Direction getDirection() { return direction; }

    @Override public void setPower(double p) {
        writes++;
        power = Math.max(-1, Math.min(1, p));
        if (mode == RunMode.RUN_USING_ENCODER) velocityTarget = power * sign() * freeTicksPerSec;
        if (!SimWorld.current.started && p != 0) commandedBeforeStart = true;
    }
    @Override public double getPower() { return power; }

    @Override public void setMode(RunMode m) {
        if (m == RunMode.STOP_AND_RESET_ENCODER) {
            encoderZero = position;
            power = 0;
            velocityTarget = 0;
            return;   // the real SDK stays in STOP_AND_RESET until another mode is set; close enough
        }
        if (m == RunMode.RUN_USING_ENCODER && mode != RunMode.RUN_USING_ENCODER) velocityTarget = power * sign() * freeTicksPerSec;
        mode = m;
    }
    @Override public RunMode getMode() { return mode; }
    @Override public void setZeroPowerBehavior(ZeroPowerBehavior z) { zeroPower = z; }
    @Override public ZeroPowerBehavior getZeroPowerBehavior() { return zeroPower; }

    @Override public int getCurrentPosition() { return (int) Math.round((position - encoderZero) * sign()); }
    @Override public void setTargetPosition(int p) { targetPosition = p; }
    @Override public int getTargetPosition() { return targetPosition; }
    @Override public boolean isBusy() { return Math.abs(getCurrentPosition() - targetPosition) > 10; }

    @Override public double getVelocity() {
        double sum = 0;
        for (double v : velHistory) sum += v;
        return sum / velHistory.length * sign();
    }

    @Override public void setVelocity(double ticksPerSecond) {
        writes++;
        mode = RunMode.RUN_USING_ENCODER;          // the SDK switches modes for you
        velocityTarget = ticksPerSecond * sign();  // stored in the motor's own sense
        power = ticksPerSecond / freeTicksPerSec;
        if (!SimWorld.current.started && ticksPerSecond != 0) commandedBeforeStart = true;
    }

    @Override public void setVelocityPIDFCoefficients(double p, double i, double d, double f) { pidfSet = true; }

    void pushVelocitySample() {
        System.arraycopy(velHistory, 0, velHistory, 1, velHistory.length - 1);
        velHistory[0] = velocity;
    }
}
