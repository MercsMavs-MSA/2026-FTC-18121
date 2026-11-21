package org.firstinspires.ftc.Maverick.Subsystems.Shooter.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.Maverick.Subsystems.Shooter.ShooterCalculator;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/**
 * Vision aim correction from the turret-mounted Limelight 3A (USB, AprilTag pipeline).
 *
 * Odometry aims the turret first. The camera then measures where the up CELL's tag row really is.
 * The difference between that and where odometry thinks the goal is becomes a small, slowly filtered
 * angle correction that Shooter adds to the turret aim. It only changes the aim: the pose and the
 * distance used for the flywheel rpm still come from odometry.
 *
 * Only the up (scorable) CELL's 4 tags are used: Robot passes that CELL's first tag ID from
 * FieldConstants (red A = 34, red B = 30, blue A = 42, blue B = 38). Tag layout from Salvadrn's
 * BIOBUZZ CellVision: 4 tags in a row centered under the CELL opening.
 *
 * Frames are only used while the turret and robot are nearly still, so camera latency (~30-50 ms)
 * can't turn motion into a false correction. Readings far from the current correction are thrown out.
 */
public class Vision {

    // ===== Camera and pipeline =====
    public static String DEVICE_NAME = "limelight";   // name in the robot configuration
    public static int PIPELINE = 0;                   // AprilTag pipeline slot in the Limelight web UI
    public static int POLL_RATE_HZ = 100;

    // ===== Camera mounting on the turret. PLACEHOLDERS: measure on the robot =====
    public static double CAMERA_FORWARD_IN = 0.0;      // in front of the turret's rotation axis, along the shooter direction
    public static double CAMERA_LEFT_IN = 0.0;         // left of the turret's rotation axis: 0, centered on the turret
    public static double CAMERA_YAW_OFFSET_DEG = 2;  // camera points this far left (+) of the shot direction: 0, points with the shooter
    public static double CAMERA_PITCH_DEG = 40.0;      // camera tilted up (+) from horizontal
    // The camera is mounted upside down, but the Limelight's own orientation setting is set to upside down,
    // so tx / ty already arrive the right way up. Keep this FALSE, or they get flipped twice.
    // Only set true if you put the Limelight's orientation setting back to normal.
    public static boolean CAMERA_UPSIDE_DOWN = false;

    // ===== Correction behaviour =====
    public static int MIN_TAGS = 2;                    // tags of the up CELL needed to use a frame
    public static double GAIN = 0.15;                  // fraction of each new reading applied (smoothing)
    public static double OUTLIER_DEG = 12.0;           // ignore readings this far from the current correction
    public static double MAX_CORRECTION_DEG = 15.0;
    public static double MAX_TURRET_SPEED_DEG_S = 20.0;
    public static double MAX_ROBOT_TURN_DEG_S = 30.0;
    public static double MAX_FRAME_AGE_MS = 100.0;

    /** Each tag's position along its CELL's row, inches from the row center. */
    public static final double[] TAG_ROW_POSITION_IN = {-6.5, -2.75, 2.75, 6.5};
    public static final int TAGS_PER_CELL = 4;

    private final Limelight3A limelight;
    private final ElapsedTime clock = new ElapsedTime();

    private double correctionRad = 0.0;
    private double lastFrameTimestamp = Double.NaN;
    private double lastHeading = Double.NaN, lastTime = 0;
    private double lastUsedS = -1e9;
    private double lastReadingDeg = Double.NaN;
    private double lastTagTxDeg = Double.NaN;   // tx of one up-CELL tag after the upside-down flip: + = tag right of center
    private int tagsSeen = 0, used = 0, rejected = 0;
    private String idsSeen = "-";          // every tag ID in the last frame, for diagnosing
    private int expectedFirstId = -1;      // first tag ID of the up CELL, for telemetry
    private String status = "no frames yet";

    public Vision(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, DEVICE_NAME);
        limelight.setPollRateHz(POLL_RATE_HZ);   // before start()
        limelight.pipelineSwitch(PIPELINE);
        limelight.start();
    }

    /**
     * Call once per loop after the pose update, before Shooter.update().
     * @param x, y, headingRad  robot pose from the Pinpoint (Pedro frame, inches / radians)
     * @param turretAngleRad    turret angle now (0 = zero stop)
     * @param turretVelRadS     turret speed now
     * @param targetX, targetY  the aim point Shooter is using
     * @param cellFirstTagId    first tag ID of the up CELL (FieldConstants.aimTagFirstId)
     */
    public void update(double x, double y, double headingRad, double turretAngleRad, double turretVelRadS,
                       double targetX, double targetY, int cellFirstTagId) {
        double now = clock.seconds();
        double turnRateDegS = Double.isNaN(lastHeading) ? 0.0
                : Math.toDegrees(wrap(headingRad - lastHeading)) / Math.max(1e-3, now - lastTime);
        lastHeading = headingRad;
        lastTime = now;

        expectedFirstId = cellFirstTagId;
        LLResult result = limelight.getLatestResult();
        if (result == null) { status = "no result from the Limelight"; return; }
        if (result.getTimestamp() == lastFrameTimestamp) return;   // same frame as last loop
        lastFrameTimestamp = result.getTimestamp();

        // Use the tag list directly (not isValid()): we filter by tag ID ourselves below.
        List<LLResultTypes.FiducialResult> all = result.getFiducialResults();
        idsSeen = describeIds(all);
        if (all == null || all.isEmpty()) { status = "no AprilTags in this frame"; tagsSeen = 0; return; }

        double ageMs = Math.max(0, result.getStaleness()) + result.getCaptureLatency() + result.getTargetingLatency();
        if (ageMs > MAX_FRAME_AGE_MS) { status = "frame too old"; return; }
        if (Math.abs(Math.toDegrees(turretVelRadS)) > MAX_TURRET_SPEED_DEG_S) { status = "waiting: turret moving"; return; }
        if (Math.abs(turnRateDegS) > MAX_ROBOT_TURN_DEG_S) { status = "waiting: robot turning"; return; }

        // Only the up CELL's tags.
        List<LLResultTypes.FiducialResult> tags = new ArrayList<>();
        for (LLResultTypes.FiducialResult f : all) {
            int id = f.getFiducialId();
            if (id >= cellFirstTagId && id < cellFirstTagId + TAGS_PER_CELL) tags.add(f);
        }
        tagsSeen = tags.size();
        if (!tags.isEmpty()) lastTagTxDeg = (CAMERA_UPSIDE_DOWN ? -1.0 : 1.0) * tags.get(0).getTargetXDegreesNoCrosshair();
        if (tags.size() < MIN_TAGS) { status = "up CELL: " + tags.size() + " of " + MIN_TAGS + " tags in view"; return; }

        // Where the shooter points on the field, and where the camera sits.
        double shotHeading = headingRad + turretAngleRad + ShooterCalculator.TURRET_ZERO_OFFSET_RAD;
        double pivotX = x + ShooterCalculator.TURRET_OFFSET_FORWARD_IN * Math.cos(headingRad) - ShooterCalculator.TURRET_OFFSET_LEFT_IN * Math.sin(headingRad);
        double pivotY = y + ShooterCalculator.TURRET_OFFSET_FORWARD_IN * Math.sin(headingRad) + ShooterCalculator.TURRET_OFFSET_LEFT_IN * Math.cos(headingRad);
        double camX = pivotX + CAMERA_FORWARD_IN * Math.cos(shotHeading) - CAMERA_LEFT_IN * Math.sin(shotHeading);
        double camY = pivotY + CAMERA_FORWARD_IN * Math.sin(shotHeading) + CAMERA_LEFT_IN * Math.cos(shotHeading);

        // Odometry's bearing to the goal vs the camera's measured bearing to the tag row center.
        double predicted = Math.atan2(targetY - camY, targetX - camX);
        double measured = shotHeading + Math.toRadians(CAMERA_YAW_OFFSET_DEG) + rowCenterLeftRad(tags, cellFirstTagId);
        double reading = wrap(measured - predicted);
        lastReadingDeg = Math.toDegrees(reading);

        if (Math.abs(Math.toDegrees(reading - correctionRad)) > OUTLIER_DEG) {
            rejected++;
            status = String.format("rejected: %.1f deg from current correction", Math.toDegrees(reading - correctionRad));
            return;
        }

        correctionRad += GAIN * (reading - correctionRad);
        double max = Math.toRadians(MAX_CORRECTION_DEG);
        correctionRad = Math.max(-max, Math.min(max, correctionRad));
        used++;
        lastUsedS = now;
        status = "tracking";
    }

    /** Add to the turret aim, radians (+ = turn left / CCW). */
    public double getCorrectionRad() { return correctionRad; }

    /** True if a frame was used in the last second. */
    public boolean hasRecentFix() { return clock.seconds() - lastUsedS < 1.0; }

    /** Forget the correction: call after the pose or heading is reset, or the target CELL changes. */
    public void reset() {
        correctionRad = 0.0;
        lastReadingDeg = Double.NaN;
        status = "reset";
    }

    public void stop() { limelight.stop(); }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Vision", "%s | correction %+.1f deg | last reading %s | tags %d | used %d, rejected %d",
                status, Math.toDegrees(correctionRad),
                Double.isNaN(lastReadingDeg) ? "-" : String.format("%+.1f deg", lastReadingDeg),
                tagsSeen, used, rejected);
        telemetry.addData("Vision tag tx", Double.isNaN(lastTagTxDeg) ? "-"
                : String.format("%+.1f deg (+ = tag right of image center)", lastTagTxDeg));
        telemetry.addData("Vision tags", "seen %s | using %s", idsSeen,
                expectedFirstId < 0 ? "-" : expectedFirstId + "-" + (expectedFirstId + TAGS_PER_CELL - 1));
        try {
            LLStatus st = limelight.getStatus();
            telemetry.addData("Vision camera", "pipeline %d (%s), %.0f fps | code expects pipeline %d",
                    st.getPipelineIndex(), st.getPipelineType(), st.getFps(), PIPELINE);
        } catch (Exception e) {
            telemetry.addData("Vision camera", "status unavailable: %s", e.getMessage());
        }
    }

    private static String describeIds(List<LLResultTypes.FiducialResult> tags) {
        if (tags == null || tags.isEmpty()) return "none";
        StringBuilder sb = new StringBuilder();
        for (LLResultTypes.FiducialResult f : tags) {
            if (sb.length() > 0) sb.append(",");
            sb.append(f.getFiducialId());
        }
        return sb.toString();
    }

    /**
     * Horizontal angle from the camera's center line to the center of the tag row, left-positive.
     * Fits tan(angle) = a + b * rowPosition through the tags seen; a is the row center, so it works with
     * any 2+ of the 4 tags. The camera pitch is undone first.
     */
    private static double rowCenterLeftRad(List<LLResultTypes.FiducialResult> tags, int cellFirstTagId) {
        double p = Math.toRadians(CAMERA_PITCH_DEG);
        double flip = CAMERA_UPSIDE_DOWN ? -1.0 : 1.0;
        double n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (LLResultTypes.FiducialResult f : tags) {
            double xn = Math.tan(Math.toRadians(flip * f.getTargetXDegreesNoCrosshair()));   // tx: + = right
            double yn = Math.tan(Math.toRadians(flip * f.getTargetYDegreesNoCrosshair()));   // ty: + = up
            double azRight = Math.atan2(xn, Math.cos(p) - yn * Math.sin(p));
            double v = Math.tan(-azRight);
            double pos = TAG_ROW_POSITION_IN[f.getFiducialId() - cellFirstTagId];
            n++; sx += pos; sy += v; sxx += pos * pos; sxy += pos * v;
        }
        double denom = n * sxx - sx * sx;
        double a = Math.abs(denom) < 1e-9 ? sy / n : (sy * sxx - sx * sxy) / denom;
        return Math.atan(a);
    }

    private static double wrap(double a) {
        return a - 2.0 * Math.PI * Math.floor((a + Math.PI) / (2.0 * Math.PI));
    }
}