package org.firstinspires.ftc.Maverick.Subsystems.Transfer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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

    private final DcMotorEx transfer;
    private final Servo gateServo;

    private State state = State.OFF;
    private boolean manualGateOpen = false;
    private boolean gateEnabled = false;   // false until start(): nothing moves in INIT

    private double lastPower = Double.NaN;
    private double lastGatePosition = Double.NaN;
    private boolean gateOpen = false;

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
                power = TRANSFER_MAX_POWER;
                open = false;
                break;
            case FEEDING:
                power = TRANSFER_MAX_POWER;
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
            if (position != lastGatePosition) {
                gateServo.setPosition(position);
                lastGatePosition = position;
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