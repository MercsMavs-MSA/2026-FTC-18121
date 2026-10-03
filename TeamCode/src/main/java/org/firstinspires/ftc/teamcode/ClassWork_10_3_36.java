package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Classwork_10_3_36", group = "Robot")

public class ClassWork_10_3_36 extends OpMode {
    DcMotor testDrive;
    public void init() {
        testDrive = hardwareMap.get(DcMotor.class, "test_drive");
//        testDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    @Override
    public void loop() {
        testDrive.setPower(-0.7);
    }
}
