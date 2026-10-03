package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Aarav_Motor", group = "Robot")

public class Aarav_Motor extends OpMode {
    DcMotor frontLeftDrive;

    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
    }
    @Override
    public void loop() {
        frontLeftDrive.setPower(-0.7);
    }
}
