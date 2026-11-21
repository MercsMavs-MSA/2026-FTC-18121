package sim;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.Maverick.Subsystems.Intake.Intake;
import org.firstinspires.ftc.Maverick.Subsystems.Shooter.ShooterCalculator;
import org.firstinspires.ftc.Maverick.Subsystems.Transfer.Transfer;

import java.util.ArrayList;
import java.util.List;

/**
 * The simulated robot and field. Every number in the PHYSICS block is an estimate, not a measurement:
 * they are good enough to check logic (states, gating, limits, signs), not to tune gains.
 */
public class SimWorld {

    // ===================== PHYSICS (estimates; change them to explore) =====================
    public static double DT = 0.02;                       // 50 Hz loop
    public static double SERVO_TRAVEL_S = 0.10;           // time for a servo to reach a new position

    public static double FLYWHEEL_FREE_TPS = 6000 / 60.0 * 28;   // 6000 rpm motor, 28 ticks: 2800 ticks/s
    public static double FLYWHEEL_TICKS_PER_WHEEL_REV = 28 * 1.5; // true value used to report wheel rpm
    public static double FLYWHEEL_LOADED_MAX = 0.92;      // fraction of free speed reachable with the wheel on
    public static double FLYWHEEL_SPINUP_TAU = 0.35;      // s, how fast the hub loop closes on a target
    public static double FLYWHEEL_COAST_TAU = 3.0;        // s, coasting down with FLOAT
    public static double FLYWHEEL_BRAKE_TAU = 0.25;       // s, if something brakes it
    public static double BALL_DIP_FRACTION = 0.12;        // wheel speed lost per ball

    public static double TURRET_FREE_TPS = 1620 / 60.0 * 103.8;   // 1620 rpm motor encoder ticks/s
    public static double TURRET_TICKS_PER_REV = 103.8 * 200.0 / 48.0;
    public static double TURRET_TAU = 0.05;
    public static double TURRET_STATIC_POWER = 0.06;      // power below which friction holds it still
    public static double TURRET_HARD_STOP_DEG = 190;      // physical stop, a bit past the soft limits

    public static double DRIVE_MAX_IN_S = 160;            // ~1150 rpm / 1.5 on ~100 mm wheels, free
    public static double DRIVE_MAX_TURN_RAD_S = 9;
    public static double DRIVE_TAU = 0.12;
    // Wheel mounting signs. Chosen so the OLD Rooster stick mapping (forward = left_stick_y) drives forward,
    // since that code "worked perfectly". Your new chassis may differ: check on blocks.
    public static double LEFT_MOUNT = +1, RIGHT_MOUNT = -1;

    public static double GATE_TO_WHEEL_S = 0.12;          // ball travel from gate to flywheel
    public static double BALL_SPACING_S = 0.25;           // how fast balls follow each other through an open gate
    public static double INTAKE_BALL_EVERY_S = 0.40;
    public static double OUTTAKE_BALL_EVERY_S = 0.30;
    public static int MAX_BALLS = 3;

    // ===================== state =====================
    public static SimWorld current;
    public static long nowNanos() { return current == null ? 0 : current.nanos; }

    public long nanos = 0;
    public boolean started = false;

    public double x, y, heading;               // TRUE pose, field inches / radians (x forward at heading 0, y left)
    public final double[] wheelFrac = new double[4];   // fl, fr, bl, br physical, fraction of max

    public final HardwareMap hardwareMap = new HardwareMap();
    public final SimMotor fl, fr, bl, br, intake, transfer, flywheel, turret;
    public final SimServo flower, gate;
    public final GoBildaPinpointDriver pinpoint;
    public final LynxModule controlHub = new LynxModule(), expansionHub = new LynxModule();

    public int balls = 0;
    public boolean fieldBallsAvailable = true;
    private double intakeTimer = 0, outtakeTimer = 0, nextFeedAt = 0;
    private final List<Double> inFlight = new ArrayList<>();

    public double targetX, targetY;            // where shots are scored against (set by the scenario)
    public final List<Shot> shots = new ArrayList<>();
    public boolean flywheelBraked = false;
    public boolean turretHitHardStop = false;
    public double maxAbsTurretDeg = 0;
    public int gateOpenEvents = 0;
    private boolean lastGateOpen = false;

    public static class Shot {
        public double t, wheelRpm, targetRpm, rpmErrPct, aimErrDeg, aimTolDeg, distanceIn;
        public boolean rpmOk() { return Math.abs(rpmErrPct) <= 3.0; }
        public boolean aimOk() { return Math.abs(aimErrDeg) <= aimTolDeg; }
    }

    public SimWorld() {
        current = this;
        fl = motor("frontLeftDrive", 1150 / 60.0 * 145.1);
        fr = motor("frontRightDrive", 1150 / 60.0 * 145.1);
        bl = motor("backLeftDrive", 1150 / 60.0 * 145.1);
        br = motor("backRightDrive", 1150 / 60.0 * 145.1);
        intake = motor("intakeMotor", 1000);
        transfer = motor("transferMotor", 1000);
        flywheel = motor("flywheelMotor", FLYWHEEL_FREE_TPS);
        turret = motor("turretMotor", TURRET_FREE_TPS);
        flower = new SimServo("flowerServo");
        gate = new SimServo("gateServo");
        hardwareMap.put("flowerServo", flower);
        hardwareMap.put("gateServo", gate);
        pinpoint = new GoBildaPinpointDriver(this);
        hardwareMap.put("pinpoint", pinpoint);
        hardwareMap.putUnnamed(controlHub);
        hardwareMap.putUnnamed(expansionHub);
    }

    private SimMotor motor(String name, double freeTps) {
        SimMotor m = new SimMotor(name, freeTps);
        hardwareMap.put(name, m);
        return m;
    }

    // ===================== helpers =====================
    public double wheelRpm() { return flywheel.velocity / FLYWHEEL_TICKS_PER_WHEEL_REV * 60.0; }
    public double flywheelTargetRpm() {
        return flywheel.mode == DcMotor.RunMode.RUN_USING_ENCODER ? flywheel.velocityTarget / FLYWHEEL_TICKS_PER_WHEEL_REV * 60.0 : 0;
    }
    public double turretDeg() { return turret.position / TURRET_TICKS_PER_REV * 360.0; }
    public boolean gatePhysicallyOpen() {
        if (Double.isNaN(gate.physical)) return false;
        return Math.abs(gate.physical - Transfer.GATE_OPEN_POSITION) < Math.abs(gate.physical - Transfer.GATE_CLOSED_POSITION);
    }
    public boolean gateCommandedOpen() {
        if (Double.isNaN(gate.commanded)) return false;
        return Math.abs(gate.commanded - Transfer.GATE_OPEN_POSITION) < Math.abs(gate.commanded - Transfer.GATE_CLOSED_POSITION);
    }
    private static boolean inward(double power, double inwardConstant) {
        return power != 0 && Math.signum(power) == Math.signum(inwardConstant);
    }
    private static boolean outward(double power, double inwardConstant) {
        return power != 0 && Math.signum(power) != Math.signum(inwardConstant);
    }

    // ===================== physics step =====================
    public void step() {
        double dt = DT;
        nanos += (long) (dt * 1e9);

        stepFlywheel(dt);
        stepTurret(dt);
        stepDrive(dt);
        flower.step();
        gate.step();
        stepBalls(dt);

        for (SimMotor m : new SimMotor[]{fl, fr, bl, br, intake, transfer, flywheel, turret}) m.pushVelocitySample();

        boolean open = gatePhysicallyOpen();
        if (open && !lastGateOpen) gateOpenEvents++;
        lastGateOpen = open;
    }

    private static double approach(double v, double target, double tau, double dt) {
        return v + (target - v) * (1 - Math.exp(-dt / tau));
    }

    private void stepFlywheel(double dt) {
        SimMotor m = flywheel;
        double max = FLYWHEEL_FREE_TPS * FLYWHEEL_LOADED_MAX;
        if (m.mode == DcMotor.RunMode.RUN_USING_ENCODER) {
            double tgt = Math.max(-max, Math.min(max, m.velocityTarget));
            if (tgt == 0) {
                if (Math.abs(m.velocity) > 100) flywheelBraked = true;   // hub actively holding 0 = braking
                m.velocity *= Math.exp(-dt / FLYWHEEL_BRAKE_TAU);
            } else {
                m.velocity = approach(m.velocity, tgt, FLYWHEEL_SPINUP_TAU, dt);
            }
        } else {
            double u = m.appliedPower();
            if (u == 0) {
                double tau = m.zeroPower == DcMotor.ZeroPowerBehavior.FLOAT ? FLYWHEEL_COAST_TAU : FLYWHEEL_BRAKE_TAU;
                if (m.zeroPower != DcMotor.ZeroPowerBehavior.FLOAT && Math.abs(m.velocity) > 100) flywheelBraked = true;
                m.velocity *= Math.exp(-dt / tau);
            } else {
                m.velocity = approach(m.velocity, u * max, FLYWHEEL_SPINUP_TAU, dt);
            }
        }
        m.position += m.velocity * dt;
    }

    private void stepTurret(double dt) {
        SimMotor m = turret;
        double sign = m.direction == DcMotor.Direction.REVERSE ? -1 : 1;
        if (m.mode == DcMotor.RunMode.RUN_TO_POSITION) {
            double tgtPos = m.targetPosition * sign + m.encoderZero;
            double cap = Math.abs(m.power) * TURRET_FREE_TPS;
            double vt = Math.max(-cap, Math.min(cap, 5.0 * (tgtPos - m.position)));
            m.velocity = approach(m.velocity, vt, TURRET_TAU, dt);
        } else {
            double u = m.appliedPower();
            if (Math.abs(u) < TURRET_STATIC_POWER && Math.abs(m.velocity) < 0.02 * TURRET_FREE_TPS) {
                m.velocity = 0;
            } else if (u == 0) {
                m.velocity *= Math.exp(-dt / (m.zeroPower == DcMotor.ZeroPowerBehavior.BRAKE ? 0.02 : 0.3));
            } else {
                m.velocity = approach(m.velocity, u * TURRET_FREE_TPS, TURRET_TAU, dt);
            }
        }
        m.position += m.velocity * dt;
        double deg = turretDeg();
        if (Math.abs(deg) > TURRET_HARD_STOP_DEG) {
            turretHitHardStop = true;
            m.position = Math.signum(deg) * TURRET_HARD_STOP_DEG / 360.0 * TURRET_TICKS_PER_REV;
            m.velocity = 0;
        }
        maxAbsTurretDeg = Math.max(maxAbsTurretDeg, Math.abs(turretDeg()));
    }

    private void stepDrive(double dt) {
        SimMotor[] w = {fl, fr, bl, br};
        double[] mount = {LEFT_MOUNT, RIGHT_MOUNT, LEFT_MOUNT, RIGHT_MOUNT};
        for (int i = 0; i < 4; i++) {
            double u = w[i].appliedPower() * mount[i];
            wheelFrac[i] = approach(wheelFrac[i], u, DRIVE_TAU, dt);
            w[i].velocity = wheelFrac[i] * mount[i] * w[i].freeTicksPerSec;
            w[i].position += w[i].velocity * dt;
        }
        double pFL = wheelFrac[0], pFR = wheelFrac[1], pBL = wheelFrac[2], pBR = wheelFrac[3];
        double forward = (pFL + pFR + pBL + pBR) / 4 * DRIVE_MAX_IN_S;
        double strafeRight = (pFL - pFR - pBL + pBR) / 4 * DRIVE_MAX_IN_S;
        double turnCw = (pFL - pFR + pBL - pBR) / 4 * DRIVE_MAX_TURN_RAD_S;
        double vx = forward, vy = -strafeRight, omega = -turnCw;
        double c = Math.cos(heading), s = Math.sin(heading);
        x += (vx * c - vy * s) * dt;
        y += (vx * s + vy * c) * dt;
        heading += omega * dt;
    }

    private void stepBalls(double dt) {
        double now = nanos / 1e9;
        boolean intakeIn = inward(intake.power, Intake.INTAKE_MAX_POWER);
        boolean intakeOut = outward(intake.power, Intake.INTAKE_MAX_POWER);
        boolean transferIn = inward(transfer.power, Transfer.TRANSFER_MAX_POWER);
        boolean transferOut = outward(transfer.power, Transfer.TRANSFER_MAX_POWER);

        if (intakeIn && transferIn && fieldBallsAvailable && balls < MAX_BALLS) {
            intakeTimer += dt;
            if (intakeTimer >= INTAKE_BALL_EVERY_S) { balls++; intakeTimer = 0; }
        } else {
            intakeTimer = 0;
        }

        if (intakeOut && transferOut && balls > 0) {
            outtakeTimer += dt;
            if (outtakeTimer >= OUTTAKE_BALL_EVERY_S) { balls--; outtakeTimer = 0; }
        } else {
            outtakeTimer = 0;
        }

        if (gatePhysicallyOpen() && transferIn && balls > 0 && now >= nextFeedAt) {
            balls--;
            inFlight.add(now + GATE_TO_WHEEL_S);
            nextFeedAt = now + BALL_SPACING_S;
        }

        for (int i = inFlight.size() - 1; i >= 0; i--) {
            if (inFlight.get(i) <= now) {
                inFlight.remove(i);
                recordShot(now);
                flywheel.velocity *= (1 - BALL_DIP_FRACTION);
            }
        }
    }

    private void recordShot(double now) {
        Shot s = new Shot();
        s.t = now;
        s.wheelRpm = wheelRpm();
        s.targetRpm = flywheelTargetRpm();
        s.rpmErrPct = s.targetRpm > 0 ? (s.wheelRpm - s.targetRpm) / s.targetRpm * 100 : -100;
        double c = Math.cos(heading), sn = Math.sin(heading);
        double px = x + ShooterCalculator.TURRET_OFFSET_FORWARD_IN * c - ShooterCalculator.TURRET_OFFSET_LEFT_IN * sn;
        double py = y + ShooterCalculator.TURRET_OFFSET_FORWARD_IN * sn + ShooterCalculator.TURRET_OFFSET_LEFT_IN * c;
        s.distanceIn = Math.hypot(targetX - px, targetY - py);
        double bearing = Math.atan2(targetY - py, targetX - px);
        double pointing = heading + Math.toRadians(turretDeg()) + ShooterCalculator.TURRET_ZERO_OFFSET_RAD;
        s.aimErrDeg = Math.toDegrees(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians(pointing - bearing));
        s.aimTolDeg = Math.toDegrees(Math.atan2(Math.max(0, ShooterCalculator.TARGET_HALF_WIDTH_IN - ShooterCalculator.BALL_RADIUS_IN),
                Math.max(s.distanceIn, 1)));
        shots.add(s);
    }
}
