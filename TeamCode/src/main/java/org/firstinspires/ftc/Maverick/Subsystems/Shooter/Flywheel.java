package org.firstinspires.ftc.Maverick.Subsystems.Shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Flywheel {

    public static double MAX_RPM = 4000.0;
    public static double TICKS_PER_REV = 42;

    public static boolean USE_PIDF = false;

    public static double P = 10.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 11.7;

    public static double RPM_THRESHOLD = 60.0;
    public static double RPM_DEADBAND = 5.0;

    private final DcMotorEx shooterMotor;
    private final ElapsedTime shootTimer = new ElapsedTime();

    private double targetRPM = 0;
    private double rpm = 0;
    private double sentRpm = Double.NaN;
    private boolean velocityMode = false;
    private boolean inRange = false;
    private boolean ready = false;

    public Flywheel(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        coast();
    }

    public void setTargetRpm(double wheelRpm) {
        targetRPM = Range.clip(wheelRpm, 0, MAX_RPM);
    }

    public void stop() {
        targetRPM = 0;
        ready = false;
        inRange = false;
        coast();
    }

    public void applyPidf() {
        if (velocityMode && USE_PIDF) {
            shooterMotor.setVelocityPIDFCoefficients(P, I, D, F);
        }
    }

    public void update() { //call every teleop loop
        rpm = shooterMotor.getVelocity() / TICKS_PER_REV * 60.0;

        if (targetRPM <= 0) {
            coast();
            inRange = false;
            ready = false;
            return;
        }

        if (!velocityMode) {
            shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            velocityMode = true;
            sentRpm = Double.NaN;
            applyPidf();
        }
        if (Double.isNaN(sentRpm) || Math.abs(targetRPM - sentRpm) >= RPM_DEADBAND) {
            shooterMotor.setVelocity(targetRPM / 60.0 * TICKS_PER_REV);
            sentRpm = targetRPM;
        }

        double range = RPM_THRESHOLD;
        boolean now = Math.abs(rpm - targetRPM) <= range;
        if (now && !inRange) shootTimer.reset();
        inRange = now;
        ready = inRange && shootTimer.milliseconds() >= 100.0;
    }

    private void coast() {
        if (!velocityMode && sentRpm == 0) return;   // already coasting
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setPower(0);
        velocityMode = false;
        sentRpm = 0;
    }

    public boolean atSpeed()      { return ready; }
    public boolean isSpinningUp() { return targetRPM > 0 && !ready; }
    public double  getRpm()       { return rpm; }
    public double  getTargetRpm() { return targetRPM; }

}
