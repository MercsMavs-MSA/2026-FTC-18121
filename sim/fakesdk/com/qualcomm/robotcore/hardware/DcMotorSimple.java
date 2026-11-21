package com.qualcomm.robotcore.hardware;

public interface DcMotorSimple {
    enum Direction { FORWARD, REVERSE }
    void setDirection(Direction direction);
    Direction getDirection();
    void setPower(double power);
    double getPower();
}
