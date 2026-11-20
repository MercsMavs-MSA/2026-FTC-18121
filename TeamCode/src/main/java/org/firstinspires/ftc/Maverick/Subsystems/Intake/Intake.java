package org.firstinspires.ftc.Maverick.Subsystems.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    public enum State {
        OFF,
        INTAKING,
        REVERSING
    }

    public static double INTAKE_MAX_POWER = -1.0;  // same as the old Rooster code
    public static double REVERSE_POWER = 1.0;      // opposite sign of INTAKE_MAX_POWER

    public static double FLOWER_OPEN_POSITION = 0.8;    // same as the old Rooster code
    public static double FLOWER_CLOSED_POSITION = 0.2;

    private final DcMotor intake;
    private final Servo flowerServo;

    private State state = State.OFF;
    private double lastPower = Double.NaN;

    private boolean flowerOpen = true;
    private boolean flowerEnabled = false;   // false until start(): nothing moves in INIT
    private double lastFlowerPosition = Double.NaN;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intakeMotor");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(0);

        flowerServo = hardwareMap.get(Servo.class, "flowerServo");
    }

    //call at START, not INIT
    public void start() {
        flowerEnabled = true;
        flowerOpen = true;
    }

    public void setState(State newState) {
        state = newState;
    }

    public State getState() {
        return state;
    }

    public void setFlowerOpen(boolean open) {
        flowerOpen = open;
    }

    public void toggleFlower() {
        flowerOpen = !flowerOpen;
    }

    public boolean isFlowerOpen() {
        return flowerOpen;
    }

    public void update() {
        double power;
        switch (state) {
            case INTAKING:
                power = INTAKE_MAX_POWER;
                break;
            case REVERSING:
                power = REVERSE_POWER;
                break;
            default:
                power = 0.0;
                break;
        }
        if (power != lastPower) {
            intake.setPower(power);
            lastPower = power;
        }

        if (flowerEnabled) {
            double position = flowerOpen ? FLOWER_OPEN_POSITION : FLOWER_CLOSED_POSITION;
            if (position != lastFlowerPosition) {
                flowerServo.setPosition(position);
                lastFlowerPosition = position;
            }
        }
    }

    public void stop() {
        state = State.OFF;
        update();
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake", state);
        telemetry.addData("Flower", flowerOpen ? "OPEN" : "CLOSED");
    }
}