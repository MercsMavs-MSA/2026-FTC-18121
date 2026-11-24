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
 *   flower-only auto (Pedro). Start (56, 8).
 *   Path 1  shooter charges + aims on the way. Arrive, wait, shoot 4.
 *   Path 2  drive, then flower grabber DOWN.
 *   Path 3  intake on while driving, wait 1 s, grabber UP, intake off.
 *   Path 4, 5  drive.
 *   Path 6  shooter charges + aims on the way. Arrive, wait 1 s, shoot 4.
 *   Then grabber DOWN, intake on, path 7, wait 1 s, grabber UP, intake off.
 *   Path 8  shooter charges + aims on the way. Arrive, wait 1 s, shoot 4.
 *   Path 9  park.
 */
@Autonomous(name = "Red: Flower Double Tip + Park", group = "Autonomous", preselectTeleOp = "18121 Maverick TeleOp")
public class FLOWER_DoubleTip_AndParkAuto extends LinearOpMode {

    // ===== Waits (seconds) =====
    public static double FIRST_AIM_WAIT_S = 0.5;    // after path 1, before shooting - @nitin we need to prob tune this
    public static double AIM_WAIT_S = 1.0;          // after paths 6 and 8, before shooting - @nitin we need to prob tune this
    public static double FLOWER_MOVE_S = 0.4;       // time for the grabber servo to finish moving - @nitin we need to prob tune this
    public static double FLOWER_INTAKE_S = 1.0;     // intake time for the balls at the flower - @nitin we need to prob tune this

    public static double FIRST_SHOT_S = 2.0;        // first ball: gate opens, back-off, wait, feed
    public static double BETWEEN_SHOTS_S = 0.5;     // gate open, transfer stopped, before each next ball - @nitin we need to prob tune this
    public static double SHOT_PULSE_S = 1.3;        // transfer runs this long to send one ball - @nitin we need to prob tune this
    public static int VOLLEY_1_BALLS = 4;
    public static int VOLLEY_2_BALLS = 4;
    public static int VOLLEY_3_BALLS = 4;

    public static boolean USE_FAR_RPM = false;

    public static boolean FLOWER_DOWN_IS_OPEN = false;

    private static final FieldConstants.Alliance ALLIANCE = FieldConstants.Alliance.RED;

    private Follower follower;
    private List<LynxModule> hubs;
    private Drive drive;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Vision vision;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(56.3773, 23.8655, 90);
    private final Pose point2 = poseFactory.of(17.7374, 46.6041, 149.5243);
    private final Pose point3 = poseFactory.of(7.8222, 46.5181, -179.5032);
    private final Pose point4 = poseFactory.of(28.6601, 55.91, 180);
    private final Pose point5 = poseFactory.of(41.5847, 118.2483, -101.7132);
    private final Pose point6 = poseFactory.of(54.5234, 116.5973, 270);
    private final Pose point7 = poseFactory.of(47.1589, 132.1956, 90);
    private final Pose point8 = poseFactory.of(47.5327, 115.4494, 90);
    private final Pose point9 = poseFactory.of(4.8242, 116.0978, -0.8698);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                // ---- Path 1: shoot 4 ----
                instant(this::spinUpShooter),
                follow(follower, path1()),
                waitMs(ms(FIRST_AIM_WAIT_S)),
                shoot(VOLLEY_1_BALLS),
                instant(shooter::idleShooter),

                // ---- Path 2, then grabber down ----
                follow(follower, path2()),
                instant(this::flowerDown),
                waitMs(ms(FLOWER_MOVE_S)),

                // ---- Path 3: intake on, wait, grabber up, intake off ----
                instant(this::startIntaking),
                follow(follower, path3()),
                waitMs(ms(FLOWER_INTAKE_S)),
                instant(this::flowerUp),
                instant(this::ballPathOff),

                // ---- Paths 4 and 5 ----
                follow(follower, path4()),
                follow(follower, path5()),

                // ---- Path 6: shoot 4 ----
                instant(this::spinUpShooter),
                follow(follower, path6()),
                waitMs(ms(AIM_WAIT_S)),
                shoot(VOLLEY_2_BALLS),
                instant(shooter::idleShooter),

                // ---- Grabber down, path 7 with intake on, wait, grabber up, intake off ----
                instant(this::flowerDown),
                waitMs(ms(FLOWER_MOVE_S)),
                instant(this::startIntaking),
                follow(follower, path7()),
                waitMs(ms(FLOWER_INTAKE_S)),
                instant(this::flowerUp),
                instant(this::ballPathOff),

                // ---- Path 8: shoot 4 ----
                instant(this::spinUpShooter),
                follow(follower, path8()),
                waitMs(ms(AIM_WAIT_S)),
                shoot(VOLLEY_3_BALLS),
                instant(shooter::idleShooter),

                // ---- Path 9: park ----
                follow(follower, path9())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();

        hubs = hardwareMap.getAll(LynxModule.class);
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

        drive.saveForTeleop();
        shooter.getTurret().saveForTeleop();
        shooter.stop();
        intake.stop();
        transfer.stop();
        if (vision != null) vision.stop();
    }


    private void spinUpShooter() {
        shooter.startShooter();
    }

    private void startIntaking() {
        intake.setState(Intake.State.INTAKING);
        transfer.setState(Transfer.State.INTAKING);
    }

    private void flowerDown() {
        intake.setFlowerOpen(FLOWER_DOWN_IS_OPEN);
    }

    private void flowerUp() {
        intake.setFlowerOpen(!FLOWER_DOWN_IS_OPEN);
    }

    private static long ms(double seconds) {
        return Math.round(seconds * 1000.0);
    }

    private void ballPathOff() {
        intake.setState(Intake.State.OFF);
        transfer.setState(Transfer.State.OFF);   // gate closes
    }

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

    private void feedBall() {
        intake.setState(Intake.State.INTAKING);
        transfer.setState(Transfer.State.FEEDING);
    }

    private void pauseBetweenBalls() {
        intake.setState(Intake.State.OFF);
        transfer.setState(Transfer.State.HOLD_OPEN);
    }


    private void syncPose() {
        drive.setCachedPose(follower.pose().x(), follower.pose().y(), follower.pose().heading());
    }

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

    // Paths from pedro

    public Path path1() {
        return Paths.line(start, path1).constant(path1);
    }

    public Path path2() {
        return Paths.line(path1, point2).tangent();
    }

    public Path path3() {
        return Paths.line(point2, point3).tangent();
    }

    public Path path4() {
        return Paths.line(point3, point4).constant(point4);
    }

    public Path path5() {
        return Paths.line(point4, point5).reverseTangent();
    }

    public Path path6() {
        return Paths.line(point5, point6).constant(point6);
    }

    public Path path7() {
        return Paths.line(point6, point7).constant(point7);
    }

    public Path path8() {
        return Paths.line(point7, point8).constant(point8);
    }

    public Path path9() {
        return Paths.line(point8, point9).reverseTangent();
    }
}