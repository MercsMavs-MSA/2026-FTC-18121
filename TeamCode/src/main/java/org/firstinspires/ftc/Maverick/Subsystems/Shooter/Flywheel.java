package org.firstinspires.ftc.Maverick.Subsystems.Shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Flywheel {

    public static double MAX_RPM = 4000.0;
    public static double TICKS_PER_REV = 42;

    public static boolean USE_PIDF = false;

    public static double P = 10.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 11.7;

    // TEMPORARY: ignore the target rpm and run the wheel at full power. "At speed" then means the wheel
    // has stopped speeding up (gained less than RPM_THRESHOLD over the last SETTLE_WINDOW_MS) and is
    // within RPM_THRESHOLD of its recent top speed (so a ball's dip counts as not ready). Set false to go
    // back to closed-loop rpm control from the shot table.
    public static boolean FULL_POWER_MODE = false;
    public static double FULL_POWER_MIN_RPM = 1000.0;       // never "at speed" below this
    public static double PEAK_DECAY_RPM_PER_S = 150.0;      // lets the reference follow a sagging battery
    public static double SETTLE_WINDOW_MS = 150.0;

    public static double RPM_THRESHOLD = 60.0;
    public static double RPM_DEADBAND = 5.0;

    private final DcMotorEx shooterMotor;
    private final ElapsedTime shootTimer = new ElapsedTime();

    private double targetRPM = 0;
    private double rpm = 0;
    private double sentRpm = Double.NaN;
    private boolean velocityMode = false;
    private boolean inRange = false;
    private boolean ready = false;
    private double peakRpm = 0;
    private final ElapsedTime peakTimer = new ElapsedTime();
    private final ElapsedTime clock = new ElapsedTime();
    private final java.util.ArrayDeque<double[]> history = new java.util.ArrayDeque<>();   // {ms, rpm}

    public Flywheel(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        coast();
    }

    public void setTargetRpm(double wheelRpm) {
        targetRPM = Range.clip(wheelRpm, 0, MAX_RPM);
    }

    public void stop() {
        targetRPM = 0;
        ready = false;
        inRange = false;
        coast();
    }

    public void applyPidf() {
        if (velocityMode && USE_PIDF) {
            shooterMotor.setVelocityPIDFCoefficients(P, I, D, F);
        }
    }

    public void update() { //call every teleop loop
        rpm = shooterMotor.getVelocity() / TICKS_PER_REV * 60.0;

        if (targetRPM <= 0) {
            coast();
            inRange = false;
            ready = false;
            peakRpm = 0;
            history.clear();
            return;
        }

        if (FULL_POWER_MODE) {
            if (velocityMode || sentRpm != -1) {
                shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                shooterMotor.setPower(1.0);
                velocityMode = false;
                sentRpm = -1;   // marks "full power is on"
            }
            double dt = Math.min(peakTimer.seconds(), 0.1);
            peakTimer.reset();
            peakRpm = Math.max(rpm, peakRpm - PEAK_DECAY_RPM_PER_S * dt);

            double now = clock.milliseconds();
            history.addLast(new double[]{now, rpm});
            while (history.size() > 1 && now - history.peekFirst()[0] > SETTLE_WINDOW_MS) history.removeFirst();
            boolean windowFull = now - history.peekFirst()[0] >= SETTLE_WINDOW_MS * 0.8;
            double gained = rpm - history.peekFirst()[1];

            boolean nowFull = rpm >= FULL_POWER_MIN_RPM
                    && windowFull && gained < RPM_THRESHOLD          // stopped speeding up
                    && rpm >= peakRpm - RPM_THRESHOLD;               // not in a ball's dip
            if (nowFull && !inRange) shootTimer.reset();
            inRange = nowFull;
            ready = inRange && shootTimer.milliseconds() >= 100.0;
            return;
        }

        if (!velocityMode) {
            shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            velocityMode = true;
            sentRpm = Double.NaN;
            applyPidf();
        }
        if (Double.isNaN(sentRpm) || Math.abs(targetRPM - sentRpm) >= RPM_DEADBAND) {
            shooterMotor.setVelocity(targetRPM / 60.0 * TICKS_PER_REV);
            sentRpm = targetRPM;
        }

        double range = RPM_THRESHOLD;
        boolean now = Math.abs(rpm - targetRPM) <= range;
        if (now && !inRange) shootTimer.reset();
        inRange = now;
        ready = inRange && shootTimer.milliseconds() >= 100.0;
    }

    private void coast() {
        if (!velocityMode && sentRpm == 0) return;   // already coasting
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setPower(0);
        velocityMode = false;
        sentRpm = 0;
    }

    public boolean atSpeed()      { return ready; }
    public boolean isSpinningUp() { return targetRPM > 0 && !ready; }
    public double  getRpm()       { return rpm; }
    public double  getTargetRpm() { return targetRPM; }

}
