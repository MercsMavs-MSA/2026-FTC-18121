package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// Code two motors, make a hardware map, make one motor reverse
// make right bumper push both motors max throttle
@TeleOp(name="Second Assignment")
public class LakshTestCode extends OpMode {
    DcMotor firstMotor;
    DcMotor secondMotor;

    @Override
    public void init() {
        firstMotor = hardwareMap.get(DcMotor.class, "first_motor");
        secondMotor = hardwareMap.get(DcMotor.class, "second_motor");

        firstMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
        firstMotor.setPower(1);
        secondMotor.setPower(1);
        } else {
      firstMotor.setPower(0);
      secondMotor.setPower(0);
        }
    }
}



