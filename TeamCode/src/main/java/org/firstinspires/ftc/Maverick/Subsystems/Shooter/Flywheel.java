package org.firstinspires.ftc.Maverick.Subsystems.Shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Flywheel {

    // GoBilda 5203 6000 rpm motor (28 encoder ticks per motor turn), belt STEP-UP: wheel turns 1.5x the motor.
    // All rpm values in this file are TRUE WHEEL rpm.
    public static double MAX_RPM = 4000.0;               // 6000 motor rpm x 1.5 (no load)
    public static double TICKS_PER_REV = 28.0 * 1.5;     // 18.67 encoder ticks per wheel turn

    /**
     * HUB_PIDF:  the hub's built-in velocity loop with P/I/D/F below.
     * BANG_BANG: our own loop. Below the target by more than BANG_BAND_RPM: FULL power (fastest possible
     *            recovery after a ball). Above it by more than BANG_BAND_RPM: power off (coast, never brakes).
     *            In between: feedforward (battery-compensated) + HOLD_KP, so it holds speed without chattering.
     *            BANG_BAND_RPM = 0 gives pure bang-bang.
     */
    public enum Control { HUB_PIDF, BANG_BANG }
    public static Control CONTROL = Control.BANG_BANG;

    // ===== BANG_BANG settings (true wheel rpm) =====
    public static double BANG_BAND_RPM = 90.0;   // sim: 200 held 3 quick balls within 0.5%; 0 (pure bang-bang) overshot ~4%
    public static double FF_TOP_RPM = 3600.0;   // MEASURE: wheel rpm at full power (FULL_POWER_MODE) on a charged battery
    public static double FF_VOLTAGE = 12.5;     // battery voltage when FF_TOP_RPM was measured
    public static double HOLD_KP = 0.0017;      // extra power per rpm below target inside the band

    // ===== HUB_PIDF settings =====
    public static boolean USE_PIDF = true;

    public static double P = 270.0;
    public static double I = 0.0;
    public static double D = 10.0;
    public static double F = 11.7;   // 32767 / 2800 ticks/s motor top speed. Re-measure: 32767 / (full-power wheel rpm * TICKS_PER_REV / 60)

    // TEMPORARY: ignore the target rpm and run the wheel at full power. "At speed" then means the wheel
    // has stopped speeding up (gained less than RPM_THRESHOLD over the last SETTLE_WINDOW_MS) and is
    // within RPM_THRESHOLD of its recent top speed (so a ball's dip counts as not ready). Set false to go
    // back to closed-loop rpm control from the shot table.
    public static boolean FULL_POWER_MODE = false;
    public static double FULL_POWER_MIN_RPM = 1000.0;       // never "at speed" below this
    public static double PEAK_DECAY_RPM_PER_S = 150.0;      // lets the reference follow a sagging battery
    public static double SETTLE_WINDOW_MS = 150.0;

    public static double RPM_THRESHOLD = 60.0;   // "at speed" band (same tolerance as the old 60 in the old units)
    public static double RPM_DEADBAND = 5.0;
    /** When the TARGET drops by more than this (e.g. back to the idle roll after shooting), coast down
     *  instead of letting the velocity loop brake the wheel with reverse power. Only a target drop starts
     *  a coast-down, so a recovery overshoot in the middle of a burst never cuts power. */
    public static double COAST_DOWN_RPM = 150.0;

    private final DcMotorEx shooterMotor;
    private final ElapsedTime shootTimer = new ElapsedTime();

    private double targetRPM = 0;
    private double rpm = 0;
    private double sentRpm = Double.NaN;
    private boolean velocityMode = false;
    private boolean inRange = false;
    private boolean ready = false;
    private double peakRpm = 0;
    private double lastTargetRpm = 0;
    private boolean coastingDown = false;
    private final ElapsedTime peakTimer = new ElapsedTime();
    private final ElapsedTime clock = new ElapsedTime();
    private final java.util.ArrayDeque<double[]> history = new java.util.ArrayDeque<>();   // {ms, rpm}

    private VoltageSensor battery = null;
    private double volts = 12.5;
    private final ElapsedTime voltTimer = new ElapsedTime();
    private double appliedPower = 0.0;

    public Flywheel(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        for (VoltageSensor v : hardwareMap.voltageSensor) {
            if (v.getVoltage() > 0) { battery = v; break; }
        }
        if (battery != null) volts = battery.getVoltage();
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
        if (targetRPM < lastTargetRpm - COAST_DOWN_RPM) coastingDown = true;   // target dropped a lot
        lastTargetRpm = targetRPM;

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
                appliedPower = 1.0;
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

        if (CONTROL == Control.BANG_BANG) {
            updateBangBang();
            return;
        }

        if (coastingDown && rpm > targetRPM + COAST_DOWN_RPM) {
            coast();          // target was lowered: let the wheel slow down on its own
            inRange = false;
            ready = false;
            return;
        }
        coastingDown = false;

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

    private void updateBangBang() {
        if (velocityMode) {   // coming from HUB_PIDF: switch to direct power
            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            velocityMode = false;
        }
        if (battery != null && voltTimer.milliseconds() > 250) {   // reading the battery is slow: 4x a second is plenty
            double v = battery.getVoltage();
            if (v > 6) volts = v;
            voltTimer.reset();
        }

        double error = targetRPM - rpm;   // + = wheel is slow
        double power;
        if (error > BANG_BAND_RPM) {
            power = 1.0;                  // slow: full power, recover as fast as the motor can
        } else if (error < -BANG_BAND_RPM) {
            power = 0.0;                  // fast: coast down, never brake
        } else {
            power = targetRPM / FF_TOP_RPM * (FF_VOLTAGE / volts) + HOLD_KP * error;
        }
        power = Range.clip(power, 0.0, 1.0);
        if (Double.isNaN(appliedPower) || Math.abs(power - appliedPower) >= 0.005 || (power == 0) != (appliedPower == 0)
                || (power == 1) != (appliedPower == 1)) {
            shooterMotor.setPower(power);
            appliedPower = power;
        }
        sentRpm = -2;   // marks "our own loop is driving the motor"

        boolean now = Math.abs(error) <= RPM_THRESHOLD;
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
        appliedPower = 0.0;
    }

    public boolean atSpeed()      { return ready; }
    public boolean isSpinningUp() { return targetRPM > 0 && !ready; }
    public double  getRpm()       { return rpm; }
    public double  getTargetRpm() { return targetRPM; }
    /** Motor power we last applied (BANG_BANG / full power), 0..1. NaN-free; 0 when the hub's loop is driving. */
    public double  getAppliedPower() { return velocityMode ? 0.0 : appliedPower; }
    public double  getBatteryVolts() { return volts; }

}