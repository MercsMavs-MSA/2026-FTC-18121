package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class AaravTestCode extends OpMode {
    DcMotor motor1;
    DcMotor motor2;

    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");

        motor1.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if(gamepad1.right_bumper) {
            motor1.setPower(1);
            motor2.setPower(1);

        }
        else{
            motor1.setPower(0);
            motor2.setPower(0);
        }
    }
}
