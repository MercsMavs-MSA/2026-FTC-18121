package org.firstinspires.ftc.Maverick.Subsystems.Transfer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Transfer {

    public enum State {
        OFF,
        INTAKING,
        TAP_FEED,    // TeleOp operator: transfer pushes while the button is held. Gate is NOT touched (it's a toggle).
        FEEDING,     // autos: opens the gate itself, backs off, waits, then feeds (the sequence below)
        HOLD_OPEN,   // autos: gate stays open, transfer stopped (the pause between shots, after FEEDING)
        REVERSING
    }

    public static double TRANSFER_MAX_POWER = 1.0;
    public static double TRANSFER_REVERSE_POWER = -1.0;

    public static double GATE_CLOSED_POSITION = 0.185;
    public static double GATE_OPEN_POSITION = 0.26;

    // TeleOp tap feed: transfer power while the operator holds the feed button.
    public static double TAP_FEED_POWER = 0.55;
    // If intaking starts while the gate is open, the transfer waits this long for the gate to close first,
    // so the ball sitting at the gate isn't pushed into the flywheel.
    public static double GATE_CLOSE_WAIT_MS = 200.0;

    // ===== FEEDING (autos only). Times are from the moment the gate starts opening =====
    // 1. BACK OFF: run the transfer in reverse for BACKOFF_MS to pull the stored pollen down off the gate.
    public static double BACKOFF_MS = 120.0;
    public static double BACKOFF_POWER = -0.4;      // negative = reverse
    // 2. WAIT: transfer off until FEED_DELAY_MS, so the gate finishes moving out of the way.
    public static double FEED_DELAY_MS = 700.0;     // must be >= BACKOFF_MS (if not, feeding starts right after the back-off)
    // 3. FEED: push the pollen into the shooter.
    public static double FEED_POWER = 0.55;

    // Re-send the gate position this often, so a command that didn't take (gate closed onto a ball, got
    // knocked, servo glitched) is retried instead of waiting for the next state change.
    public static double GATE_REFRESH_MS = 100.0;
    // The SDK may skip a command identical to the last one, so each refresh alternates the position by
    // this much to force it out. 0.0005 is about 1 microsecond of servo pulse: far too small to move the gate.
    public static double GATE_DITHER = 0.0005;

    private final DcMotorEx transfer;
    private final Servo gateServo;

    private State state = State.OFF;
    private boolean gateToggledOpen = false;   // the operator's gate toggle; intaking / outtaking clear it
    private boolean gateEnabled = false;   // false until start(): nothing moves in INIT

    private double lastPower = Double.NaN;
    private double lastGatePosition = Double.NaN;
    private boolean gateOpen = false;
    private final ElapsedTime gateRefreshTimer = new ElapsedTime();
    private final ElapsedTime gateOpenTimer = new ElapsedTime();   // time since the gate was told to open
    private final ElapsedTime gateCloseTimer = new ElapsedTime();  // time since the gate was told to close
    private boolean dither = false;

    public Transfer(HardwareMap hardwareMap) {
        transfer = hardwareMap.get(DcMotorEx.class, "transferMotor");
        transfer.setDirection(DcMotor.Direction.REVERSE);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setPower(0);

        gateServo = hardwareMap.get(Servo.class, "gateServo");
    }

    public void start() {
        gateEnabled = true;
        gateToggledOpen = false;
    }

    public void setState(State newState) {
        state = newState;
    }

    public State getState() {
        return state;
    }

    /** Operator gate toggle. Stays open until toggled again, or until the transfer intakes / outtakes (closes it). */
    public void toggleGate() {
        gateToggledOpen = !gateToggledOpen;
    }

    public void setGateOpen(boolean open) {
        gateToggledOpen = open;
    }

    public boolean isGateOpen() {
        return gateOpen;
    }

    /** True once the gate is open and the back-off and wait are done, so balls are being pushed through. */
    public boolean isFeedReady() {
        return gateOpen && gateOpenTimer.milliseconds() >= Math.max(FEED_DELAY_MS, BACKOFF_MS);
    }

    public void update() {
        double power;
        boolean open;
        switch (state) {
            case INTAKING:
                power = TRANSFER_MAX_POWER*0.75;
                open = false;              // intaking stacks the balls against the closed gate
                gateToggledOpen = false;   // and cancels the operator's toggle: it stays closed afterwards
                break;
            case TAP_FEED:
                power = TAP_FEED_POWER;
                open = gateToggledOpen;
                break;
            case FEEDING:
                power = FEED_POWER;   // the back-off / wait phases below override this at the start
                open = true;
                gateToggledOpen = false;   // the gate closes again when feeding stops
                break;
            case HOLD_OPEN:
                power = 0.0;               // gate stays open, motor stopped; no back-off or wait when FEEDING resumes
                open = true;
                gateToggledOpen = false;
                break;
            case REVERSING:
                power = TRANSFER_REVERSE_POWER;
                open = false;
                gateToggledOpen = false;
                break;
            default:
                power = 0.0;
                open = gateToggledOpen;
                break;
        }

        // gateOpen still holds last loop's value here: a false -> true change starts the sequence
        if (open && !gateOpen) gateOpenTimer.reset();
        if (!open && gateOpen) gateCloseTimer.reset();
        // intaking right after the gate was open: hold the transfer until the gate has closed
        if (state == State.INTAKING && (gateOpen || gateCloseTimer.milliseconds() < GATE_CLOSE_WAIT_MS)) {
            power = 0.0;
        }
        if (state == State.FEEDING) {
            double t = gateOpenTimer.milliseconds();
            if (t < BACKOFF_MS) {
                power = BACKOFF_POWER;                         // 1. pull the pollen down off the gate
            } else if (t < Math.max(FEED_DELAY_MS, BACKOFF_MS)) {
                power = 0.0;                                   // 2. let the gate finish opening
            }                                                  // 3. after that: FEED_POWER
        }

        if (power != lastPower) {
            transfer.setPower(power);
            lastPower = power;
        }

        gateOpen = open;
        if (gateEnabled) {
            double position = open ? GATE_OPEN_POSITION : GATE_CLOSED_POSITION;
            // send on a change right away, and re-send every GATE_REFRESH_MS while it stays the same
            if (position != lastGatePosition || gateRefreshTimer.milliseconds() >= GATE_REFRESH_MS) {
                dither = !dither;
                gateServo.setPosition(position + (dither ? GATE_DITHER : 0.0));
                lastGatePosition = position;
                gateRefreshTimer.reset();
            }
        }
    }

    public void stop() {
        state = State.OFF;
        gateToggledOpen = false;
        update();
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Transfer", state);
        String gate;
        if (!gateOpen) gate = "CLOSED";
        else if (state != State.FEEDING) gate = "OPEN";
        else if (isFeedReady()) gate = "OPEN (feeding)";
        else if (state == State.FEEDING && gateOpenTimer.milliseconds() < BACKOFF_MS) gate = "OPENING (backing off)";
        else gate = "OPENING";
        telemetry.addData("Gate", gate);
    }
}