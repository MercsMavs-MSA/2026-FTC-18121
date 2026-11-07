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

// Import Constants from the pedro sub-package (matching your working RED opmode)
import org.firstinspires.ftc.RoosterDrive_Version1.pedro.Constants;

@Autonomous(name = "PedroPathing_auto_v1_blue", group = "Autonomous")
public class PedroPathing_auto_v1_blue extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(84.8549, 131.5582, 270);
    private final Pose path1 = poseFactory.of(85.2248, 105.1778, 270);
    private final Pose point2Start = poseFactory.of(85.2248, 105.1778, 270);
    private final Pose point2 = poseFactory.of(121.1709, 138.4347, 0);
    private final Pose point3 = poseFactory.of(138.3192, 138.1923, -0.8099);
    private final Pose point4Start = poseFactory.of(138.3192, 138.1923, -112);
    private final Pose point4 = poseFactory.of(90.2877, 17.9179, 90);
    private final Pose point5Start = poseFactory.of(90.2877, 17.9179, 90);
    private final Pose point5 = poseFactory.of(83.7019, 31.8502, 90);
    private final Pose point6 = poseFactory.of(94.3891, 1.983, -70.3115);
    private final Pose point7 = poseFactory.of(89.2795, 28.0296, 101.099);
    private final Pose point8 = poseFactory.of(137.7396, 92.9932, 53.2786);
    private final Pose point9Start = poseFactory.of(137.7396, 92.9932, 53);
    private final Pose point9 = poseFactory.of(84.1748, 138.9661, 270);

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
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point2, point3).tangent();
    }

    public Path path4() {
        return Paths.line(point4Start, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.line(point5Start, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point5, point6).tangent();
    }

    public Path path7() {
        return Paths.line(point6, point7).tangent();
    }

    public Path path8() {
        return Paths.line(point7, point8).tangent();
    }

    public Path path9() {
        return Paths.line(point9Start, point9).linear(point9Start, point9);
    }
}