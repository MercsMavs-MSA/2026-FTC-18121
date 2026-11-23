package org.firstinspires.ftc.Maverick.Autos;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class DoubleTip_AndParkAuto extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(7.2324, 30.5366, 90);
    private final Pose path1 = poseFactory.of(51.5666, 29.7193, 90);
    private final Pose point2Start = poseFactory.of(51.5666, 29.7193, 90);
    private final Pose point2 = poseFactory.of(5.8476, 12.4725, 270);
    private final Pose point3 = poseFactory.of(5.6341, 4.6181, 270);
    private final Pose point4Start = poseFactory.of(5.6341, 4.6181, 270);
    private final Pose point4 = poseFactory.of(47.0101, 116.321, 270);
    private final Pose point5 = poseFactory.of(46.9248, 135.4121, 90.2559);
    private final Pose point6Start = poseFactory.of(46.9248, 135.4121, 90);
    private final Pose point6 = poseFactory.of(48.345, 116.324, 270);
    private final Pose point7 = poseFactory.of(28.6892, 96.3077, -134.4794);
    private final Pose point8 = poseFactory.of(7.9098, 96.5068, -0.549);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6()),
                follow(follower, path7()),
                follow(follower, path8())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = org.firstinspires.ftc.RoosterDrive_Version1.pedro.Constants.create(hardwareMap);
        assert follower != null;
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return Paths.line(start, path1).constant(path1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point2, point3).constant(point3);
    }

    public Path path4() {
        return Paths.line(point4Start, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.line(point4, point5).tangent();
    }

    public Path path6() {
        return Paths.line(point6Start, point6).linear(point6Start, point6);
    }

    public Path path7() {
        return Paths.line(point6, point7).tangent();
    }

    public Path path8() {
        return Paths.line(point7, point8).reverseTangent();
    }
}
