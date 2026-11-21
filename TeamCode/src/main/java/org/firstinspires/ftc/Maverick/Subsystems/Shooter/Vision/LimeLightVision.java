//package org.firstinspires.ftc.Maverick.Subsystems.Shooter.Vision;
//
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.Maverick.Subsystems.Shooter.ShooterCalculator;
//import org.firstinspires.ftc.Maverick.Subsystems.Shooter.Turret;
//
//import java.util.List;
//
///*
// * Limelight 3A, AprilTag pipeline only, for BIOBUZZ 2026-27. Robot.java just calls this class.
// * The Limelight is NOT used for localization: odometry does the main aiming, tx only trims the turret.
// *
// * Robot.java usage:
// *   init:      vision = new LimeLightVision(hardwareMap); vision.setAlliance(BLUE or RED);
// *   start:     vision.start();
// *   each loop: vision.update(drive.getX(), drive.getY(), drive.getHeadingRad(), turret, aimTrim, rpmTrim);
// *              flywheel.setTargetRpm(vision.getTargetRpm());
// *              if (driverPressedShoot && vision.canShoot(flywheel.atSpeed())) { feed the ball }
// *
// * Logic:
// *  - Our alliance has 2 CELLS (audience side, opposite side), each with a 4-tag cluster.
// *  - The CELL on the robot's side is the one nearest the robot (from odometry).
// *  - Aim: odometry + ShooterCalculator points the turret at that CELL; when its tags are visible,
// *    the average tx of the cluster corrects the turret.
// *  - Gate: no shot unless tags of our CELL on the robot's side are confidently seen. No tags seen,
// *    or only the other alliance's tags seen, means no shot.
// *
// * Assumes the Limelight is on the turret and can see the CELL underside tags. Not run on a robot.
// */
//public class LimeLightVision {
//
//    public enum Alliance { RED, BLUE }
//
//    // Tag IDs from the TU09 manual, section 9.9. Index 0 = audience-side CELL, 1 = opposite-side CELL.
//    // NOTE: the manual lists red as 30-37. If your field uses 0-7, change them here.
//    public static int[] BLUE_AUDIENCE_TAGS = {38, 39, 40, 41};
//    public static int[] BLUE_OPPOSITE_TAGS = {42, 43, 44, 45};
//    public static int[] RED_AUDIENCE_TAGS = {34, 35, 36, 37};
//    public static int[] RED_OPPOSITE_TAGS = {30, 31, 32, 33};
//
//    public static int APRILTAG_PIPELINE = 0;
//
//    // CELL opening positions in your odometry frame (inches). PLACEHOLDERS: set from your field frame.
//    public static double BLUE_AUD_X = 0.0, BLUE_AUD_Y = 0.0;
//    public static double BLUE_OPP_X = 0.0, BLUE_OPP_Y = 0.0;
//    public static double RED_AUD_X = 0.0, RED_AUD_Y = 0.0;
//    public static double RED_OPP_X = 0.0, RED_OPP_Y = 0.0;
//
//    public static double MAX_STALENESS_MS = 100.0;
//    public static int CONFIRM_LOOPS = 3;
//    public static double TX_FILTER = 0.6;
//    public static double TX_DEADBAND_DEG = 0.3;
//    public static double TX_BIAS_DEG = 0.0;
//
//    private final Limelight3A limelight;
//    private Alliance alliance = Alliance.BLUE;
//
//    private final double[] sumTx = new double[2];
//    private final int[] count = new int[2];
//
//    private boolean ourTagSeen = false;
//    private boolean otherTagSeen = false;
//    private int aimSide = 0;     // 0 = audience-side CELL, 1 = opposite-side CELL
//    private int seenStreak = 0;
//    private double txDeg = 0.0;
//    private boolean txInitialized = false;
//
//    private ShooterCalculator.ShotSolution solution = null;
//
//    public LimeLightVision(HardwareMap hardwareMap) {
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.pipelineSwitch(APRILTAG_PIPELINE);
//        limelight.setPollRateHz(100);
//    }
//
//    public void start() { limelight.start(); }
//    public void stop()  { limelight.stop(); }
//
//    public void setAlliance(Alliance a) { alliance = a; }
//    public Alliance getAlliance()       { return alliance; }
//
//    private int[] ourTags(int side) {
//        if (alliance == Alliance.BLUE) return side == 0 ? BLUE_AUDIENCE_TAGS : BLUE_OPPOSITE_TAGS;
//        return side == 0 ? RED_AUDIENCE_TAGS : RED_OPPOSITE_TAGS;
//    }
//
//    private int[] otherTags(int side) {
//        if (alliance == Alliance.BLUE) return side == 0 ? RED_AUDIENCE_TAGS : RED_OPPOSITE_TAGS;
//        return side == 0 ? BLUE_AUDIENCE_TAGS : BLUE_OPPOSITE_TAGS;
//    }
//
//    private double cellX(int side) {
//        if (alliance == Alliance.BLUE) return side == 0 ? BLUE_AUD_X : BLUE_OPP_X;
//        return side == 0 ? RED_AUD_X : RED_OPP_X;
//    }
//
//    private double cellY(int side) {
//        if (alliance == Alliance.BLUE) return side == 0 ? BLUE_AUD_Y : BLUE_OPP_Y;
//        return side == 0 ? RED_AUD_Y : RED_OPP_Y;
//    }
//
//    private static boolean contains(int[] ids, int id) {
//        for (int v : ids) if (v == id) return true;
//        return false;
//    }
//
//    // Call once per loop. Reads the camera, solves the shot, and commands the turret.
//    public void update(double robotX, double robotY, double robotHeadingRad,
//                       Turret turret, double aimTrimRad, double rpmTrim) {
//        // The CELL on the robot's side is the nearer one.
//        double d0 = Math.hypot(cellX(0) - robotX, cellY(0) - robotY);
//        double d1 = Math.hypot(cellX(1) - robotX, cellY(1) - robotY);
//        aimSide = d0 <= d1 ? 0 : 1;
//
//        readCamera();
//
//        solution = ShooterCalculator.solve(robotX, robotY, robotHeadingRad,
//                cellX(aimSide), cellY(aimSide), turret.getAngleRad(), aimTrimRad, rpmTrim);
//
//        double turretTarget = solution.turretAngleRad;
//        if (tagsLocked() && Math.abs(txDeg) >= TX_DEADBAND_DEG) {
//            // Camera on turret: tx is positive to the right, turret angle is positive counterclockwise.
//            turretTarget = turret.getAngleRad() - Math.toRadians(txDeg);
//        }
//        turret.setTargetAngle(turretTarget);
//        turret.update();
//    }
//
//    private void readCamera() {
//        for (int s = 0; s < 2; s++) { sumTx[s] = 0; count[s] = 0; }
//        ourTagSeen = false;
//        otherTagSeen = false;
//
//        LLResult result = limelight.getLatestResult();
//        boolean fresh = result != null && result.isValid() && result.getStaleness() <= MAX_STALENESS_MS;
//
//        if (fresh) {
//            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
//            for (LLResultTypes.FiducialResult t : tags) {
//                int id = t.getFiducialId();
//                for (int s = 0; s < 2; s++) {
//                    if (contains(ourTags(s), id)) {
//                        sumTx[s] += t.getTargetXDegrees();
//                        count[s]++;
//                        ourTagSeen = true;
//                    } else if (contains(otherTags(s), id)) {
//                        otherTagSeen = true;
//                    }
//                }
//            }
//        }
//
//        // Only tags of our CELL on the robot's side count.
//        if (count[aimSide] > 0) {
//            seenStreak++;
//            double rawTx = sumTx[aimSide] / count[aimSide] - TX_BIAS_DEG;
//            if (!txInitialized) {
//                txDeg = rawTx;
//                txInitialized = true;
//            } else {
//                txDeg += TX_FILTER * (rawTx - txDeg);
//            }
//        } else {
//            seenStreak = 0;
//            txInitialized = false;
//        }
//    }
//
//    private boolean tagsLocked() { return seenStreak >= CONFIRM_LOOPS; }
//
//    // Only the other alliance's tags are visible.
//    public boolean wrongGoalVisible() { return otherTagSeen && !ourTagSeen; }
//
//    // Flywheel rpm for the current distance; 0 when the shot isn't valid.
//    public double getTargetRpm() {
//        return (solution != null && solution.valid()) ? solution.flywheelRpm : 0.0;
//    }
//
//    // True only when: shot valid, flywheel at speed, our CELL's tags (robot's side) are confidently
//    // seen, and the camera error is within tolerance.
//    public boolean canShoot(boolean flywheelAtSpeed) {
//        if (solution == null || !solution.valid()) return false;
//        if (!flywheelAtSpeed) return false;
//        if (!tagsLocked()) return false;
//        if (wrongGoalVisible()) return false;
//        return Math.abs(Math.toRadians(txDeg)) <= solution.aimToleranceRad;
//    }
//
//    public boolean tagsVisible() { return tagsLocked(); }
//    public double getTxDeg()     { return txDeg; }
//    public int getAimSide()      { return aimSide; }
//    public ShooterCalculator.ShotSolution getSolution() { return solution; }
//}