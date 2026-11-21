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
        FEEDING,     // gate open, transfer pushing balls into the flywheel
        REVERSING
    }

    public static double TRANSFER_MAX_POWER = 1.0;
    public static double TRANSFER_REVERSE_POWER = -1.0;

    public static double GATE_CLOSED_POSITION = 0.185;
    public static double GATE_OPEN_POSITION = 0.26;

    // Re-send the gate position this often, so a command that didn't take (gate closed onto a ball, got
    // knocked, servo glitched) is retried instead of waiting for the next state change.
    public static double GATE_REFRESH_MS = 100.0;
    // The SDK may skip a command identical to the last one, so each refresh alternates the position by
    // this much to force it out. 0.0005 is about 1 microsecond of servo pulse: far too small to move the gate.
    public static double GATE_DITHER = 0.0005;

    private final DcMotorEx transfer;
    private final Servo gateServo;

    private State state = State.OFF;
    private boolean manualGateOpen = false;
    private boolean gateEnabled = false;   // false until start(): nothing moves in INIT

    private double lastPower = Double.NaN;
    private double lastGatePosition = Double.NaN;
    private boolean gateOpen = false;
    private final ElapsedTime gateRefreshTimer = new ElapsedTime();
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
        manualGateOpen = false;
    }

    public void setState(State newState) {
        state = newState;
    }

    public State getState() {
        return state;
    }

    public void toggleGate() {
        manualGateOpen = !manualGateOpen;
    }

    public boolean isGateOpen() {
        return gateOpen;
    }

    public void update() {
        double power;
        boolean open;
        switch (state) {
            case INTAKING:
                power = TRANSFER_MAX_POWER*0.75;
                open = false;
                break;
            case FEEDING:
                power = TRANSFER_MAX_POWER*0.55;
                open = true;
                break;
            case REVERSING:
                power = TRANSFER_REVERSE_POWER;
                open = false;
                break;
            default:
                power = 0.0;
                open = manualGateOpen;
                break;
        }
        if (state != State.OFF) manualGateOpen = false;

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
        manualGateOpen = false;
        update();
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Transfer", state);
        telemetry.addData("Gate", gateOpen ? "OPEN" : "CLOSED");
    }
}