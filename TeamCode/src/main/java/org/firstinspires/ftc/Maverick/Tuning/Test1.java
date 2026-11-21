package org.firstinspires.ftc.Maverick.Tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TestTransfer", group = "Robot")

public class Test1 extends OpMode {

    DcMotor transferMotor;

    @Override
    public void init() {
        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
    }

    @Override
    public void loop() {
        transferMotor.setPower(-1);
    }
}
