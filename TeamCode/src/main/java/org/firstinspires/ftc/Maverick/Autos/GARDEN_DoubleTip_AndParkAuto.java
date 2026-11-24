package org.firstinspires.ftc.Maverick.Autos;

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
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.Maverick.FieldConstants;
import org.firstinspires.ftc.Maverick.Robot;
import org.firstinspires.ftc.Maverick.Subsystems.Drive.Drive;
import org.firstinspires.ftc.Maverick.Subsystems.Intake.Intake;
import org.firstinspires.ftc.Maverick.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.Maverick.Subsystems.Shooter.Vision.Vision;
import org.firstinspires.ftc.Maverick.Subsystems.Transfer.Transfer;

import java.util.List;

/**
 * RED auto (Pedro frame), paths unchanged from the path editor:
 *   Path 1  drive to the shot spot. Shooter spins up + turret aims on the way. Wait 0.5 s, shoot the 4 preloads.
 *   Path 2  drive back to the loading corner. At its end: intake at max, gate closed.
 *   Path 3  intake the balls.
 *   Path 4  drive to the shot spot (shooter spins up + aims on the way). Wait 0.5 s, shoot.
 *   Path 5  drive to the flower. At its end: flower grabber + intake.
 *   Path 6  drive to the shot spot (shooter spins up + aims on the way). Wait 1 s, shoot 4.
 *   Path 7, 8  park.
 * The Shooter keeps spinning and aiming while its state is SHOOT, so "spin up + aim on the way" is just
 * startShooter() right before the follow(). Aiming uses the Pedro pose (fed into Drive each loop).
 * Shot timing reuses the tuned sequence from MaverickAuto (first ball, then pause + pulse per ball).
 */
@Autonomous(name = "Red: Double Tip + Park", group = "Autonomous", preselectTeleOp = "18121 Maverick TeleOp")
public class GARDEN_DoubleTip_AndParkAuto extends LinearOpMode {

    // ===== Shot timing (seconds) =====
    public static double AIM_WAIT_S = 0.5;          // wait after arriving, before shooting (volleys 1 and 2)
    public static double FINAL_AIM_WAIT_S = 1.0;    // same, for the last volley (after path 6)
    public static double FIRST_SHOT_S = 2.0;        // first ball: gate opens, back-off, wait, feed (same as MaverickAuto)
    public static double BETWEEN_SHOTS_S = 1.5;     // gate open, transfer stopped, before each next ball
    public static double SHOT_PULSE_S = 1.3;        // transfer runs this long to send one ball
    public static int VOLLEY_1_BALLS = 4;           // preloads
    public static int VOLLEY_2_BALLS = 4;           // picked up on path 3
    public static int VOLLEY_3_BALLS = 4;
    public static double FLOWER_GRAB_S = 1.0;       // time at the flower for the grabber + intake (tune)

    // false = Shooter.CLOSE_RPM, true = Shooter.FAR_RPM (the fastest shot speed the Shooter has)
    public static boolean USE_FAR_RPM = false;

    private Follower follower;
    private Drive drive;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Vision vision;

    // Paths are RED. Blue would need the paths mirrored: (x, y, h) -> (144 - x, 144 - y, h + 180).
    private static final FieldConstants.Alliance ALLIANCE = FieldConstants.Alliance.RED;

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
                // ---- Path 1: shoot the 4 preloads ----
                instant(this::spinUpShooter),
                follow(follower, path1()),
                waitMs(ms(AIM_WAIT_S)),
                shoot(VOLLEY_1_BALLS),
                instant(shooter::idleShooter),

                // ---- Path 2, then intake on + gate closed, path 3 picks up the balls ----
                follow(follower, path2()),
                instant(this::startIntaking),
                follow(follower, path3()),

                // ---- Path 4: shoot ----
                instant(this::spinUpShooter),
                follow(follower, path4()),
                waitMs(ms(AIM_WAIT_S)),
                shoot(VOLLEY_2_BALLS),
                instant(shooter::idleShooter),

                // ---- Path 5: flower ----
                follow(follower, path5()),
                instant(this::grabFlowerAndIntake),
                waitMs(ms(FLOWER_GRAB_S)),

                // ---- Path 6: shoot ----
                instant(this::spinUpShooter),
                follow(follower, path6()),
                waitMs(ms(FINAL_AIM_WAIT_S)),
                shoot(VOLLEY_3_BALLS),
                instant(shooter::idleShooter),

                // ---- Paths 7 and 8: park ----
                follow(follower, path7()),
                follow(follower, path8())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);

        drive = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap, drive);
        if (Robot.USE_VISION) vision = new Vision(hardwareMap);

        follower = org.firstinspires.ftc.RoosterDrive_Version1.pedro.Constants.create(hardwareMap);
        assert follower != null;
        follower.setPose(start);
        follower.update();

        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        syncPose();

        waitForStart();

        shooter.farShot = USE_FAR_RPM;
        shooter.getTurret().zeroAt(0.0);
        intake.start();
        transfer.start();

        schedule(autoRoutine());

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();

            follower.update();
            syncPose();
            updateAim();

            Scheduler.execute();

            shooter.update();
            intake.update();
            transfer.update();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }
            shooter.addTelemetry(telemetry);
            intake.addTelemetry(telemetry);
            transfer.addTelemetry(telemetry);

            telemetry.update();
        }

        // hand the pose and turret angle to TeleOp, then shut everything off
        drive.saveForTeleop();
        shooter.getTurret().saveForTeleop();
        shooter.stop();
        intake.stop();
        transfer.stop();
        if (vision != null) vision.stop();
    }


    // Flywheel to shot speed and turret aiming
    private void spinUpShooter() {
        shooter.startShooter();
    }

    // Intake at max, transfer stacks balls up the gate.
    private void startIntaking() {
        intake.setState(Intake.State.INTAKING);
        transfer.setState(Transfer.State.INTAKING);
    }


    private void grabFlowerAndIntake() {
        intake.setFlowerOpen(false);
        intake.setState(Intake.State.INTAKING);
    }

    private static long ms(double seconds) {
        return Math.round(seconds * 1000.0);
    }

    private void ballPathOff() {
        intake.setState(Intake.State.OFF);
        transfer.setState(Transfer.State.OFF);   // gate closes
    }

    // Shoot pollen like other
    private Command shoot(int balls) {
        Command volley = sequential(
                instant(this::feedBall),
                waitMs(ms(FIRST_SHOT_S))
        );
        for (int i = 1; i < balls; i++) {
            volley = sequential(
                    volley,
                    instant(this::pauseBetweenBalls),
                    waitMs(ms(BETWEEN_SHOTS_S)),
                    instant(this::feedBall),
                    waitMs(ms(SHOT_PULSE_S))
            );
        }
        return sequential(volley, instant(this::ballPathOff));
    }

    private void feedBall() {   // opens the gate itself, backs off, waits, then feeds
        intake.setState(Intake.State.INTAKING);
        transfer.setState(Transfer.State.FEEDING);
    }

    private void pauseBetweenBalls() {   // gate stays open, transfer stopped
        intake.setState(Intake.State.OFF);
        transfer.setState(Transfer.State.HOLD_OPEN);
    }

    // Pose / aiming

    // give PedroPathing pose to Drive so the Shooter aims from it.
    private void syncPose() {
        drive.setCachedPose(follower.pose().x(), follower.pose().y(), follower.pose().heading());
    }

    // Aim at the HIVE cell on our half of the field (A below the middle line, B above), like TeleOp.
    private void updateAim() {
        FieldConstants.HiveState cell = drive.getY() < FieldConstants.FIELD_CENTER
                ? FieldConstants.HiveState.A : FieldConstants.HiveState.B;
        double aimX = FieldConstants.aimX(ALLIANCE, cell);
        double aimY = FieldConstants.aimY(ALLIANCE, cell);
        shooter.setTarget(aimX, aimY);

        if (vision != null && Robot.USE_VISION) {
            vision.update(drive.getX(), drive.getY(), drive.getHeadingRad(),
                    shooter.getTurret().getAngleRad(), shooter.getTurret().getVelocityRadPerSec(),
                    aimX, aimY, FieldConstants.aimTagFirstId(ALLIANCE, cell));
            shooter.setAimCorrection(vision.getCorrectionRad());
        } else {
            shooter.setAimCorrection(0.0);
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Paths (unchanged)
    // ---------------------------------------------------------------------------------------------

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