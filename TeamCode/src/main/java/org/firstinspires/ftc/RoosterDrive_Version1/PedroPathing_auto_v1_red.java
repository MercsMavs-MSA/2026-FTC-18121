package org.firstinspires.ftc.RoosterDrive_Version1;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.RoosterDrive_Version1.pedro.Constants;

@Autonomous(name = "PedroPathing_auto_v1", group = "Autonomous")
public class PedroPathing_auto_v1_red extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(56, 36, 90);
    private final Pose point2 = poseFactory.of(23.5751, 2.1435, -133.7627);
    private final Pose point3 = poseFactory.of(3.0204, 1.9153, -179.3639);
    private final Pose point4 = poseFactory.of(35.229, 118.2144, 74.5201);
    private final Pose point5Start = poseFactory.of(35.229, 118.2144, 0);
    private final Pose point5 = poseFactory.of(56.9734, 117.3273, 270);
    private final Pose point6 = poseFactory.of(46.8884, 138.7055, 115.2553);
    private final Pose point7Start = poseFactory.of(46.8884, 138.7055, 115);
    private final Pose point7 = poseFactory.of(56.7394, 109.402, 270);
    private final Pose point8 = poseFactory.of(3.5637, 49.5114, -131.6012);
    private final Pose point9Start = poseFactory.of(3.5637, 49.5114, -132);
    private final Pose point9 = poseFactory.of(55.3498, 1.0633, 90);

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
                follow(follower, path8()),
                follow(follower, path9())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();

        // This will now correctly find Constants if Constants.java is in this same package folder
        follower = Constants.create(hardwareMap);
        if (follower != null) {
            follower.setPose(start);
            follower.update();
        } else {
            telemetry.addData("Error", "Follower failed to initialize!");
            telemetry.update();
        }
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
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(path1, point2).tangent();
    }

    public Path path3() {
        return Paths.line(point2, point3).tangent();
    }

    public Path path4() {
        return Paths.line(point3, point4).tangent();
    }

    public Path path5() {
        return Paths.line(point5Start, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point5, point6).tangent();
    }

    public Path path7() {
        return Paths.line(point7Start, point7).linear(point7Start, point7);
    }

    public Path path8() {
        return Paths.line(point7, point8).tangent();
    }

    public Path path9() {
        return Paths.line(point9Start, point9).linear(point9Start, point9);
    }
}