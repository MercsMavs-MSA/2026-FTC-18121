package com.qualcomm.hardware.gobilda;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Simulated Pinpoint. Reports the sim's true pose through a frame transform, so setPosition and
 * setHeading behave like re-zeroing the real device: they change what it reports, not where the robot is.
 */
public class GoBildaPinpointDriver {
    public enum GoBildaOdometryPods { goBILDA_SWINGARM_POD, goBILDA_4_BAR_POD }
    public enum EncoderDirection { FORWARD, REVERSED }

    private final sim.SimWorld world;
    private double rot = 0, tx = 0, ty = 0;   // estimate = R(rot) * truth + t ; heading = truth + rot
    private Pose2D latest = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);
    public int updates = 0;

    public GoBildaPinpointDriver(sim.SimWorld world) { this.world = world; }

    public void update() { updates++; latest = estimate(); }
    public Pose2D getPosition() { return latest; }

    public void setPosition(Pose2D p) {
        double h = p.getHeading(AngleUnit.RADIANS);
        rot = h - world.heading;
        double c = Math.cos(rot), s = Math.sin(rot);
        tx = p.getX(DistanceUnit.INCH) - (c * world.x - s * world.y);
        ty = p.getY(DistanceUnit.INCH) - (s * world.x + c * world.y);
        latest = estimate();
    }

    public void setHeading(double heading, AngleUnit unit) {
        Pose2D now = estimate();
        setPosition(new Pose2D(DistanceUnit.INCH, now.getX(DistanceUnit.INCH), now.getY(DistanceUnit.INCH),
                AngleUnit.RADIANS, unit == AngleUnit.DEGREES ? Math.toRadians(heading) : heading));
    }

    public double getHeading(AngleUnit unit) { return latest.getHeading(unit); }

    public void resetPosAndIMU() { setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0)); }
    public void setOffsets(double xOffset, double yOffset, DistanceUnit unit) {}
    public void setEncoderResolution(GoBildaOdometryPods pods) {}
    public void setEncoderResolution(double ticksPerUnit, DistanceUnit unit) {}
    public void setEncoderDirections(EncoderDirection x, EncoderDirection y) {}

    private Pose2D estimate() {
        double c = Math.cos(rot), s = Math.sin(rot);
        double ex = c * world.x - s * world.y + tx;
        double ey = s * world.x + c * world.y + ty;
        return new Pose2D(DistanceUnit.INCH, ex, ey, AngleUnit.RADIANS, AngleUnit.normalizeRadians(world.heading + rot));
    }
}
