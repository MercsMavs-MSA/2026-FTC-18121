package com.qualcomm.robotcore.hardware;

public interface DcMotorEx extends DcMotor {
    double getVelocity();
    void setVelocity(double ticksPerSecond);
    void setVelocityPIDFCoefficients(double p, double i, double d, double f);
}
