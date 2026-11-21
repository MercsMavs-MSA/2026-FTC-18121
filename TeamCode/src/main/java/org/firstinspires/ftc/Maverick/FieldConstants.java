package org.firstinspires.ftc.Maverick;

/**
 * Field coordinates for BIOBUZZ in the PEDRO PATHING frame, so poses can pass straight from a Pedro auto
 * into TeleOp.
 *
 * PEDRO FRAME (as used on visualizer.pedropathing.com):
 *   origin in the RED / AUDIENCE corner, +x toward the BLUE wall, +y toward the FAR wall (away from the
 *   audience), inches, heading 0 = facing +x, counter-clockwise positive. The field is drawn 144 x 144;
 *   the real walls are about 1.3 in inside that (141.3 in wall to wall).
 *
 * WHERE THE NUMBERS COME FROM: FTC team 12303's FieldConstants (craigmichaelmartin/ftc-12303), which they
 * took from the official field CAD in the FTC field frame, converted here with the conversion they quote
 * from the Pedro docs: x_pedro = y_ftc + 72, y_pedro = 72 - x_ftc, heading_pedro = heading_ftc - 90 deg.
 * Verify on a real field: event fields vary by about +-1 in.
 *
 * BLUE is RED turned half a turn about the field centre (the field is rotationally symmetric, not a
 * mirror): (x, y, h) -> (144 - x, 144 - y, h + 180 deg).
 */
public final class FieldConstants {

    private FieldConstants() {}

    public enum Alliance { RED, BLUE }

    /** Which CELL of our HIVE is up: A = the one up at match start, B = after a tip. */
    public enum HiveState { A, B }

    public static final double FIELD_CENTER = 72.0;

    // ---------------- Aim points (centre of the up CELL's opening), RED, Pedro frame ----------------
    // 12303: red A = FTC (16.50, -12.74), red B = FTC (-16.48, -12.74), both ~58.3 in high.
    public static final double RED_AIM_A_X = 59.26, RED_AIM_A_Y = 55.50;
    public static final double RED_AIM_B_X = 59.26, RED_AIM_B_Y = 88.48;

    /** First AprilTag ID of each red CELL (Vision): A = 34-37 (audience CELL), B = 30-33 (far CELL). Blue = +8. */
    public static final int RED_TAG_A = 34, RED_TAG_B = 30;

    // ---------------- Driver station direction ----------------
    /**
     * Field heading pointing straight AWAY from each alliance's drivers, used for field-relative driving
     * and for the heading reset. Red drivers stand at the red wall (x = 0) looking toward +x; blue drivers
     * at the blue wall looking toward -x. CHECK where your drivers actually stand.
     */
    public static final double RED_DRIVER_FORWARD_DEG = 0.0;
    public static final double BLUE_DRIVER_FORWARD_DEG = 180.0;

    // ---------------- TeleOp start pose when no auto ran (testing) ----------------
//    public static final double RED_TEST_START_X = 55.3, RED_TEST_START_Y =8.0, RED_TEST_START_DEG = 90.0;
    public static final double RED_TEST_START_X = 8.3, RED_TEST_START_Y =31.1, RED_TEST_START_DEG = 90.0;


    // ---------------- Helpers ----------------
    public static double aimX(Alliance a, HiveState s) {
        double red = (s == HiveState.A) ? RED_AIM_A_X : RED_AIM_B_X;
        return a == Alliance.RED ? red : 2 * FIELD_CENTER - red;
    }

    public static double aimY(Alliance a, HiveState s) {
        double red = (s == HiveState.A) ? RED_AIM_A_Y : RED_AIM_B_Y;
        return a == Alliance.RED ? red : 2 * FIELD_CENTER - red;
    }

    public static int aimTagFirstId(Alliance a, HiveState s) {
        int red = (s == HiveState.A) ? RED_TAG_A : RED_TAG_B;
        return a == Alliance.RED ? red : red + 8;
    }

    public static double driverForwardRad(Alliance a) {
        return Math.toRadians(a == Alliance.RED ? RED_DRIVER_FORWARD_DEG : BLUE_DRIVER_FORWARD_DEG);
    }

    public static double testStartX(Alliance a) { return a == Alliance.RED ? RED_TEST_START_X : 2 * FIELD_CENTER - RED_TEST_START_X; }
    public static double testStartY(Alliance a) { return a == Alliance.RED ? RED_TEST_START_Y : 2 * FIELD_CENTER - RED_TEST_START_Y; }
    public static double testStartDeg(Alliance a) { return a == Alliance.RED ? RED_TEST_START_DEG : RED_TEST_START_DEG + 180.0; }
}
