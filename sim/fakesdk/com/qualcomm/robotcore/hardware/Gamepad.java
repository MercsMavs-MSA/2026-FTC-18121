package com.qualcomm.robotcore.hardware;

/** Simulated gamepad. The sim script sets the fields; ...WasPressed() reports a press once, like the SDK. */
public class Gamepad {
    public float left_stick_x, left_stick_y, right_stick_x, right_stick_y;
    public float left_trigger, right_trigger;
    public boolean left_bumper, right_bumper;
    public boolean a, b, x, y;
    public boolean dpad_up, dpad_down, dpad_left, dpad_right;
    public boolean left_stick_button, right_stick_button, back, start;

    private boolean seenA, seenB, seenX, seenY, seenUp, seenDown, seenLeft, seenRight;
    private boolean seenLSB, seenRSB, seenLB, seenRB, seenBack;

    public void clear() {
        left_stick_x = left_stick_y = right_stick_x = right_stick_y = 0;
        left_trigger = right_trigger = 0;
        left_bumper = right_bumper = a = b = x = y = false;
        dpad_up = dpad_down = dpad_left = dpad_right = false;
        left_stick_button = right_stick_button = back = start = false;
    }

    public boolean aWasPressed() { boolean r = a && !seenA; seenA = a; return r; }
    public boolean bWasPressed() { boolean r = b && !seenB; seenB = b; return r; }
    public boolean xWasPressed() { boolean r = x && !seenX; seenX = x; return r; }
    public boolean yWasPressed() { boolean r = y && !seenY; seenY = y; return r; }
    public boolean dpadUpWasPressed() { boolean r = dpad_up && !seenUp; seenUp = dpad_up; return r; }
    public boolean dpadDownWasPressed() { boolean r = dpad_down && !seenDown; seenDown = dpad_down; return r; }
    public boolean dpadLeftWasPressed() { boolean r = dpad_left && !seenLeft; seenLeft = dpad_left; return r; }
    public boolean dpadRightWasPressed() { boolean r = dpad_right && !seenRight; seenRight = dpad_right; return r; }
    public boolean leftStickButtonWasPressed() { boolean r = left_stick_button && !seenLSB; seenLSB = left_stick_button; return r; }
    public boolean rightStickButtonWasPressed() { boolean r = right_stick_button && !seenRSB; seenRSB = right_stick_button; return r; }
    public boolean leftBumperWasPressed() { boolean r = left_bumper && !seenLB; seenLB = left_bumper; return r; }
    public boolean rightBumperWasPressed() { boolean r = right_bumper && !seenRB; seenRB = right_bumper; return r; }
    public boolean backWasPressed() { boolean r = back && !seenBack; seenBack = back; return r; }
}
