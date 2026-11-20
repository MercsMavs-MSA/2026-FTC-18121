package org.firstinspires.ftc.Maverick.Tuning;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.Maverick.Subsystems.Shooter.Flywheel;

import java.util.List;

//VIBE CODED TUNER THAT HOPEFULLY HELPS TUNE PID

/**
 * Bench tuner for the Flywheel class. It does not tune anything by itself. It lets you see what the
 * wheel is doing, change the gains from the gamepad, and read back numbers that say whether the
 * defaults are good enough.
 *
 * SUGGESTED ORDER (stop as soon as the answer is "good enough")
 *   1. Look at "Hub gains now". With USE_PIDF = false these are whatever the hub/SDK defaults are for
 *      the motor type in your robot configuration. This is how you find out what they actually are.
 *   2. Fresh battery, wheel clear of hands: HOLD X (flywheel must be off). It runs the motor at full
 *      power and shows the top speed after 2 s. That is the real max, not the 4000 rpm on paper. Keep
 *      real targets at about 80-85% of it, and set Flywheel.MAX_RPM to match.
 *   3. Spin up at about 70% of the measured max with the defaults (USE_PIDF false). Read "Spin-up":
 *      time to ready, overshoot, and average error once at speed. If it settles inside the window
 *      with little overshoot and a small average error, the defaults are fine.
 *   4. Feed balls at the pace the feeder will use. Read "Ball dips": the depth of each drop and how
 *      long until the wheel is back at speed. That recovery time is what limits how fast you can fire.
 *   5. Only if 3 or 4 is poor: press Y to turn custom gains on, set F to the suggested value from step 2,
 *      then raise P for faster recovery (back off if it overshoots) and trim F if it settles high or low.
 *   6. Compare a fresh battery and a tired one at the same target. A big difference is the only
 *      reason to add battery scaling.
 *   Gains you change here live only in memory: copy the final values into Flywheel.java.
 *
 * CONTROLS (gamepad1)
 *   A               flywheel on / off
 *   D-pad up/down   target rpm +/- step              B: step 50 <-> 250 rpm
 *   LB / RB         select the parameter to edit (shown with a > marker)
 *   D-pad left/right  selected parameter -/+  (P 0.5, I 0.05, D 0.05, F 0.1, window 10 rpm)
 *   Y               toggle Flywheel.USE_PIDF (see the warning in telemetry when turning it off)
 *   X (hold)        measure max speed at full power (turns the flywheel off first)
 *   Back            clear the spin-up and ball-dip stats
 *
 * Needs an SDK with Gamepad edge methods (aWasPressed etc.). Not compiled or run on a robot.
 */
@TeleOp(name = "Flywheel Tuner", group = "Tuning")
public class FlywheelTuner extends LinearOpMode {

    /** Must match the name the Flywheel class uses. */
    public static String MOTOR_NAME = "flywheelMotor";
    /** A ball = wheel falls this many rpm below the speed it was holding. */
    public static double DIP_RPM = 120.0;
    public static double MEASURE_SECONDS = 2.0;
    /** Real targets as a fraction of the measured max, for the suggestion line. */
    public static double HEADROOM = 0.85;

    private enum Param { P, I, D, F, WINDOW }

    private Flywheel flywheel;
    private DcMotorEx motor;

    // spin-up stats
    private final ElapsedTime spinTimer = new ElapsedTime();
    private boolean spinActive = false;
    private double timeToReadyMs = Double.NaN;
    private double peakRpm = 0;
    private double avgError = 0;

    // ball-dip stats
    private final ElapsedTime dipTimer = new ElapsedTime();
    private boolean armed = false, dipActive = false;
    private double heldRpm = 0, heldTarget = 0, dipMin = 0;
    private int dips = 0;
    private double lastDepth = Double.NaN, lastRecoveryMs = Double.NaN, worstRecoveryMs = Double.NaN;

    // hub gains read-back
    private final ElapsedTime gainTimer = new ElapsedTime();
    private PIDFCoefficients hubGains = null;

    @Override
    public void runOpMode() {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        flywheel = new Flywheel(hardwareMap);
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);

        Param[] params = Param.values();
        int sel = 0;
        double target = 2000;
        double step = 50;
        boolean on = false;

        boolean measuring = false;
        ElapsedTime measureTimer = new ElapsedTime();
        double measuredMaxTps = 0;

        telemetry.addLine("A on/off | dpad U/D rpm | LB/RB select | dpad L/R edit | Y USE_PIDF | hold X measure max");
        telemetry.update();
        waitForStart();

        gainTimer.reset();
        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();

            // ---------- inputs ----------
            if (gamepad1.aWasPressed()) on = !on;
            if (gamepad1.dpadUpWasPressed()) target += step;
            if (gamepad1.dpadDownWasPressed()) target -= step;
            target = Math.max(0, Math.min(Flywheel.MAX_RPM, target));
            if (gamepad1.bWasPressed()) step = (step == 50) ? 250 : 50;
            if (gamepad1.backWasPressed()) clearStats();

            if (gamepad1.leftBumperWasPressed()) sel = (sel + params.length - 1) % params.length;
            if (gamepad1.rightBumperWasPressed()) sel = (sel + 1) % params.length;
            if (gamepad1.dpadLeftWasPressed()) adjust(params[sel], -1);
            if (gamepad1.dpadRightWasPressed()) adjust(params[sel], +1);
            if (gamepad1.yWasPressed()) {
                Flywheel.USE_PIDF = !Flywheel.USE_PIDF;
                flywheel.applyPidf();
                gainTimer.reset();
            }

            // ---------- max-speed measurement (hold X) ----------
            if (gamepad1.x) {
                if (!measuring) {
                    measuring = true;
                    on = false;
                    flywheel.stop();            // leaves the Flywheel class coasting, so its state stays consistent
                    measureTimer.reset();
                    measuredMaxTps = 0;
                }
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setPower(1.0);
                if (measureTimer.seconds() >= MEASURE_SECONDS) {
                    measuredMaxTps = Math.max(measuredMaxTps, Math.abs(motor.getVelocity()));
                }
            } else if (measuring) {
                motor.setPower(0);              // FLOAT is already set, so it coasts down
                measuring = false;
            }

            // ---------- flywheel ----------
            if (!measuring) {
                double before = flywheel.getTargetRpm();
                double commanded = on ? target : 0;
                if (commanded != before) onRetarget(commanded);
                flywheel.setTargetRpm(commanded);
                flywheel.update();
                trackStats();
            }

            // ---------- hub gains read-back (a hub round trip, so not every loop) ----------
            if (hubGains == null || gainTimer.seconds() > 1.0) {
                hubGains = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                gainTimer.reset();
            }

            // ---------- telemetry ----------
            telemetry.addData("Target", "%.0f rpm (step %.0f)  %s", target, step, on ? "ON" : "off");
            telemetry.addData("Wheel", "%.0f rpm  err %+.0f  %s", flywheel.getRpm(),
                    flywheel.getRpm() - flywheel.getTargetRpm(),
                    flywheel.atSpeed() ? "AT SPEED" : (on ? "not at speed" : "off"));
            if (hubGains != null) {
                telemetry.addData("Hub gains now", "P %.3f  I %.3f  D %.3f  F %.3f", hubGains.p, hubGains.i, hubGains.d, hubGains.f);
            }
            telemetry.addData("USE_PIDF", Flywheel.USE_PIDF
                    ? "ON (custom gains sent when the wheel spins up, or after an edit while spinning)"
                    : "off (hub defaults; if you just turned this off, restart the OpMode to be sure they are the defaults)");
            for (int i = 0; i < params.length; i++) {
                telemetry.addData((i == sel ? "> " : "  ") + params[i], "%s", fmt(params[i]));
            }
            telemetry.addData("Spin-up", "ready in %s ms, overshoot %.0f rpm, avg error at speed %+.1f rpm",
                    Double.isNaN(timeToReadyMs) ? "--" : String.format("%.0f", timeToReadyMs),
                    Math.max(0, peakRpm - flywheel.getTargetRpm()), avgError);
            telemetry.addData("Ball dips", "%d seen | last depth %s rpm | recovery last %s ms, worst %s ms",
                    dips, num(lastDepth), num(lastRecoveryMs), num(worstRecoveryMs));
            if (measuring || measuredMaxTps > 0) {
                double maxRpm = measuredMaxTps / Flywheel.TICKS_PER_REV * 60.0;
                telemetry.addData("Max speed", "%.0f ticks/s = %.0f wheel rpm %s",
                        measuredMaxTps, maxRpm, measuring && measureTimer.seconds() < MEASURE_SECONDS ? "(spinning up, keep holding X)" : "");
                if (measuredMaxTps > 0) {
                    telemetry.addData("Suggested", "F = %.2f  |  keep targets <= %.0f rpm", 32767.0 / measuredMaxTps, HEADROOM * maxRpm);
                }
            }
            telemetry.update();
        }

        flywheel.stop();
        motor.setPower(0);
    }

    // ===================================================================================
    // Stats
    // ===================================================================================

    /** A new target starts a new spin-up test and ends any ball-dip episode. */
    private void onRetarget(double newTarget) {
        armed = false;
        dipActive = false;
        avgError = 0;
        if (newTarget > 0) {
            spinActive = true;
            spinTimer.reset();
            timeToReadyMs = Double.NaN;
            peakRpm = 0;
        } else {
            spinActive = false;
        }
    }

    private void trackStats() {
        double target = flywheel.getTargetRpm();
        double rpm = flywheel.getRpm();
        if (target <= 0) return;

        // spin-up: time to ready, and the peak until one second after ready
        if (spinActive) {
            peakRpm = Math.max(peakRpm, rpm);
            if (Double.isNaN(timeToReadyMs) && flywheel.atSpeed()) timeToReadyMs = spinTimer.milliseconds();
            if (!Double.isNaN(timeToReadyMs) && spinTimer.milliseconds() > timeToReadyMs + 1000) spinActive = false;
        }

        // average error once at speed (slow moving average)
        if (flywheel.atSpeed()) avgError += 0.05 * ((rpm - target) - avgError);

        // ball dips: a fall of DIP_RPM below the held speed, then time until at speed again
        if (flywheel.atSpeed()) {
            if (dipActive) {
                lastRecoveryMs = dipTimer.milliseconds();
                worstRecoveryMs = Double.isNaN(worstRecoveryMs) ? lastRecoveryMs : Math.max(worstRecoveryMs, lastRecoveryMs);
                dipActive = false;
            }
            armed = true;
            heldRpm = rpm;
            heldTarget = target;
        } else if (armed && !dipActive && rpm < heldTarget - DIP_RPM) {
            dips++;
            dipActive = true;
            armed = false;
            dipMin = rpm;
            dipTimer.reset();
        }
        if (dipActive) {
            dipMin = Math.min(dipMin, rpm);
            lastDepth = heldRpm - dipMin;
        }
    }

    private void clearStats() {
        spinActive = false;
        timeToReadyMs = Double.NaN;
        peakRpm = 0;
        avgError = 0;
        armed = false;
        dipActive = false;
        dips = 0;
        lastDepth = Double.NaN;
        lastRecoveryMs = Double.NaN;
        worstRecoveryMs = Double.NaN;
    }

    // ===================================================================================
    // Gain editing
    // ===================================================================================

    private void adjust(Param p, int dir) {
        switch (p) {
            case P: Flywheel.P = Math.max(0, r3(Flywheel.P + dir * 0.5)); break;
            case I: Flywheel.I = Math.max(0, r3(Flywheel.I + dir * 0.05)); break;
            case D: Flywheel.D = Math.max(0, r3(Flywheel.D + dir * 0.05)); break;
            case F: Flywheel.F = Math.max(0, r3(Flywheel.F + dir * 0.1)); break;
            case WINDOW: Flywheel.RPM_THRESHOLD = Math.max(10, Flywheel.RPM_THRESHOLD + dir * 10); break;
        }
        flywheel.applyPidf();   // only does anything while spinning with USE_PIDF on; otherwise applied at the next spin-up
        gainTimer.reset();
        hubGains = null;
    }

    private String fmt(Param p) {
        switch (p) {
            case P: return String.format("%.3f", Flywheel.P);
            case I: return String.format("%.3f", Flywheel.I);
            case D: return String.format("%.3f", Flywheel.D);
            case F: return String.format("%.3f", Flywheel.F);
            default: return String.format("%.0f rpm (ready window)", Flywheel.RPM_THRESHOLD);
        }
    }

    private static double r3(double v) {
        return Math.round(v * 1000.0) / 1000.0;
    }

    private static String num(double v) {
        return Double.isNaN(v) ? "--" : String.format("%.0f", v);
    }
}
