package org.firstinspires.ftc.Maverick;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "18121 Rooster Field Relative Drive", group = "Robot")
public class Robot extends OpMode {

    Limelight3A limelight;
    GoBildaPinpointDriver pinpoint;

    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    Servo flowerServo;
    Servo gateServo;

    DcMotorEx turretMotor;
    DcMotorEx shooterMotor;

    DcMotor transferMotor;
    DcMotor intakeMotor;

    @Override
    public void init() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {

    }
}
