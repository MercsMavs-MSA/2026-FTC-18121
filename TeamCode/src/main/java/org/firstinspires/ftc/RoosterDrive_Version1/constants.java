package org.firstinspires.ftc.RoosterDrive_Version1;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.pedropathing.drivetrain.Mecanum;
import com.pedropathing.drivetrain.MecanumConfig;
import com.pedropathing.localization.PinpointLocalizer;
import com.pedropathing.localization.PinpointConfig;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;

public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.backLeftName.set("backLeft");
        c.frontRightName.set("frontRight");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.xPodOffset.set(0);
        c.yPodOffset.set(0);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        // Default motion parameters
    });

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}