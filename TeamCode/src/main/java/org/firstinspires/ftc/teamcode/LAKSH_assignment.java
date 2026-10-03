package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "sixseven")
public class LAKSH_assignment extends OpMode {
    DcMotor front_drive;
    public void init() {
        front_drive = hardwareMap.get(DcMotor.class, "front_drive");
    }
@Override
    public void loop() {
        front_drive.setPower(-0.7);
}
}