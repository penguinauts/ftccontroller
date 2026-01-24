package org.firstinspires.ftc.teamcode.helpers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.function.BooleanSupplier;

@Config
public class Robot {

    // -------------------------------------------------------------------
    // ACTIVE OPMODE (SET THIS IN YOUR OPMODE: Robot.activeOpMode = this;)
    // -------------------------------------------------------------------
    public static LinearOpMode activeOpMode;

    // -------------------------------------------------------------------
    // DRIVE MOTORS
    // -------------------------------------------------------------------
    public static DcMotorEx leftWheel;
    public static DcMotorEx rightWheel;

    // -------------------------------------------------------------------
    // OTHER MOTORS
    // -------------------------------------------------------------------
    public static DcMotorEx shooter;
    public static DcMotorEx intakeMotor;

    // -------------------------------------------------------------------
    // SERVOS
    // -------------------------------------------------------------------
    public static Servo trapServo;
    public static CRServo leftGatekeeperServo;
    public static CRServo rightGatekeeperServo;

    public static IMU imu;

    // ------------------------------
    // MOTION PROFILING CONSTANTS
    // ------------------------------
    public static double MAX_VEL = 2000;     // ticks/sec (dashboard tunable)
    public static double MAX_ACCEL = 3500;   // ticks/sec^2 (dashboard tunable)
    public static double MAX_DECEL = 3000;   // ticks/sec^2 (dashboard tunable)
    public static double MIN_POWER = 0.12;   // prevents stalling at low speeds

    // -------------------------------------------------------------------
    // TUNABLE POSITIONS & POWERS
    // -------------------------------------------------------------------
    // IMPORTANT: keep these as the values that worked for you
    public static double TRAP_OPEN_POS   = 0.77;
    public static double TRAP_CLOSED_POS = 0.55;
    public static int TRAP_SEAT_MS = 140; // 120–220


    public static double INTAKE_POWER  = -1.0;
    public static double INTAKE_VELOCITY = -2500;
    public static double OUTTAKE_POWER = 0.4;

    public static double GATEKEEPER_LEFT_POWER = 1.0;
    public static double GATEKEEPER_RIGHT_POWER = 1.0;

    // -------------------------------------------------------------------
    // SHOOTER PIDF (TUNABLE)
    // -------------------------------------------------------------------
    public static double PROPORTIONAL = 80;
    public static double INTEGRAL     = 0;
    public static double DERIVATIVE   = 0;
    public static double FEED_FORWARD = 11.7;

    public static PIDFCoefficients SHOOTER_PIDF =
            new PIDFCoefficients(PROPORTIONAL, INTEGRAL, DERIVATIVE, FEED_FORWARD);

    // -------------------------------------------------------------------
    // SMART SHOOT TUNABLES (DASHBOARD)
    // -------------------------------------------------------------------
    public static double SHOOT_READY_TOL = 35;       // +/- ticks/sec
    public static int SHOOT_READY_STABLE_MS = 140;   // must be stable this long
    public static int SHOOT_READY_TIMEOUT_MS = 450;

    public static int FEED_MAX_MS = 420;             // hard cap so we never run forever
    public static int FEED_MIN_MS = 160;             // prevents premature stop
    public static int FEED_TAIL_MS = 110;            // push after dip so ball clears

    public static double DIP_FROM_PEAK = 90;        // 100–180 typical
    public static int DIP_CONFIRM_COUNT = 2;

    public static double FEED_OVERSPEED = 0;         // try 0, 40, 60 if needed

    // Velocity filtering (EMA)
    public static double SHOOTER_FILTER_ALPHA = 0.25; // 0.2–0.35

    // Optional jam recovery
    public static int JAM_REVERSE_MS = 140;
    public static int JAM_FORWARD_MS = 160;

    // -------------------------------------------------------------------
    // ENCODER VALUES
    // -------------------------------------------------------------------
    public static double WHEEL_DIAMETER_INCHES = 3.78;
    public static double TICKS_PER_INCH       = 52.2;
    public static double WHEEL_CIRCUMFERENCE  = Math.PI * WHEEL_DIAMETER_INCHES;

    public static double TURN_ROTATION_P = 150;
    //public static boolean ALLOW_OUTTAKE = true;


    public static void recomputeConstants() {
        WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_INCHES;
    }

    // -------------------------------------------------------------------
    // INITIALIZATION
    // -------------------------------------------------------------------
    public static void initializeRobot(HardwareMap hw) {

        // Gatekeepers
        leftGatekeeperServo  = hw.get(CRServo.class, "leftGatekeeperServo");
        rightGatekeeperServo = hw.get(CRServo.class, "rightGatekeeperServo");
        leftGatekeeperServo.setDirection(CRServo.Direction.REVERSE);

        // Trapdoor
        trapServo = hw.get(Servo.class, "trapServo");

        // Shooter
        shooter = hw.get(DcMotorEx.class, "Shooter");
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        SHOOTER_PIDF = new PIDFCoefficients(PROPORTIONAL, INTEGRAL, DERIVATIVE, FEED_FORWARD);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, SHOOTER_PIDF);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Intake
        intakeMotor = hw.get(DcMotorEx.class, "Intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Drive wheels
        leftWheel  = hw.get(DcMotorEx.class, "leftWheel");
        rightWheel = hw.get(DcMotorEx.class, "rightWheel");

        imu = hw.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWheel.setDirection(DcMotor.Direction.FORWARD);

        // Reset encoders
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Safe defaults
        trapServo.setPosition(TRAP_CLOSED_POS);
        setGatekeepers(0);
        setIntake(0);
    }

    // -------------------------------------------------------------------
    // SAFE WAIT
    // -------------------------------------------------------------------
    public static void safeWait(long ms) {
        if (activeOpMode == null) {
            try { Thread.sleep(ms); } catch (InterruptedException ignored) {}
            return;
        }

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (activeOpMode.opModeIsActive() && timer.milliseconds() < ms) {
            activeOpMode.idle();
        }
    }


    /**
     * Calculates the target velocity at a given position using a motion profile.
     * Supports trapezoidal and triangular profiles.
     *
     * @param targetTicks  Total distance (ticks)
     * @param currentTicks Current position (ticks)
     * @return Target velocity (ticks/sec)
     */
    public static double getProfiledVelocity(double targetTicks, double currentTicks) {
        if (targetTicks == 0) return 0;

        double x = Math.abs(currentTicks);
        double total = Math.abs(targetTicks);

        if (x > total) x = total;

        double accelDist = (MAX_VEL * MAX_VEL) / (2.0 * MAX_ACCEL);
        double decelDist = (MAX_VEL * MAX_VEL) / (2.0 * MAX_DECEL);

        double vel;

        if (accelDist + decelDist > total) {
            // TRIANGULAR PROFILE: Cannot reach MAX_VEL
            // Calculate peak velocity and transition point
            double peakVel = Math.sqrt(2.0 * MAX_ACCEL * MAX_DECEL * total / (MAX_ACCEL + MAX_DECEL));
            double transitionPoint = (peakVel * peakVel) / (2.0 * MAX_ACCEL);

            if (x < transition) {
                // Acceleration phase
                vel = Math.sqrt(2.0 * MAX_ACCEL * x);
            } else {
                // Deceleration phase
                double remain = total - x;
                vel = Math.sqrt(2.0 * MAX_DECEL * remain);
            }
        } else {
            // TRAPEZOIDAL PROFILE: Can reach MAX_VEL
            if (x < accelDist) {
                // Acceleration phase
                vel = Math.sqrt(2.0 * MAX_ACCEL * x);
            } else if (x > (total - decelDist)) {
                // Deceleration phase
                double remain = total - x;
                vel = Math.sqrt(2.0 * MAX_DECEL * remain);
            } else {
                // Constant velocity phase
                vel = MAX_VEL;
            }
        }

        // Safety clamp to ensure velocity is within bounds
        return Math.min(vel, MAX_VEL);
    }

    // -------------------------------------------------------------------
    // WAIT HELPERS
    // -------------------------------------------------------------------
    private static boolean waitUntil(BooleanSupplier cond, long timeoutMs) {
        if (activeOpMode == null) {
            long start = System.currentTimeMillis();
            while ((System.currentTimeMillis() - start) < timeoutMs) {
                if (cond.getAsBoolean()) return true;
                try { Thread.sleep(5); } catch (InterruptedException ignored) {}
            }
            return cond.getAsBoolean();
        }

        ElapsedTime t = new ElapsedTime();
        t.reset();
        while (activeOpMode.opModeIsActive() && t.milliseconds() < timeoutMs) {
            if (cond.getAsBoolean()) return true;
            activeOpMode.idle();
        }
        return cond.getAsBoolean();
    }

    // -------------------------------------------------------------------
    // ENCODER STRAIGHT DRIVE (NO IMU)
    // -------------------------------------------------------------------
    // Drift correction constant (tune if robot veers left or right)
    public static double DRIFT_CORRECTION_KP = 0.05;

    /**
     * Drives straight for a specified distance using motion profiling and drift correction.
     *
     * @param opMode The active OpMode (for checking if still active)
     * @param inches Distance to travel in inches (positive = forward, negative = backward)
     * @param maxPower Maximum power multiplier (0.0 to 1.0) to scale the motion profile
     */
    public static void driveStraightInches(LinearOpMode opMode,
                                           double inches,
                                           double maxPower) {
        // Input validation
        if (inches == 0) return;
        if (maxPower <= 0) maxPower = 0.1;
        if (maxPower > 1.0) maxPower = 1.0;

        recomputeConstants();

        int targetTicks = (int)(inches * TICKS_PER_INCH);
        double direction = Math.signum(inches);

        // Reset encoders
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Timeout protection (calculate based on distance and max velocity)
        double estimatedTime = (Math.abs(targetTicks) / MAX_VEL) * 1.5 + 2.0;
        ElapsedTime timeout = new ElapsedTime();

        while (opMode.opModeIsActive()) {
            // Check timeout
            if (timeout.seconds() > estimatedTime) {
                opMode.telemetry.addData("Warning", "Drive timeout exceeded");
                opMode.telemetry.update();
                break;
            }

            // Get individual wheel positions
            int posL = Math.abs(leftWheel.getCurrentPosition());
            int posR = Math.abs(rightWheel.getCurrentPosition());
            double avgPos = (posL + posR) / 2.0;

            // Stop when reached target distance
            if (avgPos >= Math.abs(targetTicks)) break;

            // Get desired velocity from motion profile (both params as absolute values)
            double targetVel = getProfiledVelocity(Math.abs(targetTicks), avgPos);

            // Scale the velocity directly by maxPower
            double scaledVel = targetVel * maxPower;

            // Apply MIN_POWER only when not decelerating near the end
            // (during deceleration, we want to allow lower speeds for smooth stopping)
            double remainingDist = Math.abs(targetTicks) - avgPos;
            double decelThreshold = (MIN_POWER * MIN_POWER * MAX_VEL * MAX_VEL) / (2.0 * MAX_DECEL);

            if (remainingDist > decelThreshold) {
                // Not in final deceleration zone - enforce minimum power
                double minVel = MIN_POWER * MAX_VEL;
                scaledVel = Math.max(scaledVel, minVel);
            }

            // Drift correction: compute error between left and right wheels
            int positionError = posL - posR;
            double correction = DRIFT_CORRECTION_KP * positionError;

            // Apply direction and drift correction
            double leftVel = direction * (scaledVel - correction);
            double rightVel = direction * (scaledVel + correction);

            leftWheel.setVelocity(leftVel);
            rightWheel.setVelocity(rightVel);

            opMode.idle();
        }

        stopDrive();
    }

    public static void turnDegreesIMU(LinearOpMode opMode, double degrees) {
        degrees *= -1;
        imu.resetYaw();
        double tolerance = 1;
        double velocityTolerance = 5;
        double error;

        do {
            error = degrees - imu.getRobotYawPitchRollAngles().getYaw();
            double out = TURN_ROTATION_P * error;
            rightWheel.setVelocity(out);
            leftWheel.setVelocity(-out);
        } while ((Math.abs(error) > tolerance ||
                Math.abs(imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate) > velocityTolerance)
                && opMode.opModeIsActive());

        stopDrive();
    }

    // -------------------------------------------------------------------
    // SLOW DRIVE
    // -------------------------------------------------------------------
    /**
     * Drives straight slowly for precise positioning using motion profiling and drift correction.
     * Uses reduced MIN_POWER threshold for finer control at low speeds.
     *
     * @param opMode The active OpMode (for checking if still active)
     * @param inches Distance to travel in inches (positive = forward, negative = backward)
     * @param maxPower Maximum power multiplier (0.0 to 1.0) - typically lower than regular drive
     */
    public static void driveStraightSlowInches(LinearOpMode opMode,
                                               double inches,
                                               double maxPower) {
        // Input validation
        if (inches == 0) return;
        if (maxPower <= 0) maxPower = 0.1;
        if (maxPower > 1.0) maxPower = 1.0;

        recomputeConstants();

        int targetTicks = (int)(inches * TICKS_PER_INCH);
        double direction = Math.signum(inches);

        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Timeout protection (slower movement needs more time)
        double estimatedTime = (Math.abs(targetTicks) / (MAX_VEL * maxPower)) * 2.0 + 3.0;
        ElapsedTime timeout = new ElapsedTime();

        while (opMode.opModeIsActive()) {
            // Check timeout
            if (timeout.seconds() > estimatedTime) {
                opMode.telemetry.addData("Warning", "Slow drive timeout exceeded");
                opMode.telemetry.update();
                break;
            }

            // Get individual wheel positions
            int posL = Math.abs(leftWheel.getCurrentPosition());
            int posR = Math.abs(rightWheel.getCurrentPosition());
            double avgPos = (posL + posR) / 2.0;

            if (avgPos >= Math.abs(targetTicks)) break;

            // Get desired velocity from motion profile (both params as absolute values)
            double targetVel = getProfiledVelocity(Math.abs(targetTicks), avgPos);

            // Scale the velocity directly by maxPower for slow mode
            double scaledVel = targetVel * maxPower;

            // Slow drive uses reduced MIN_POWER (80%) to allow finer control
            // Apply MIN_POWER only when not decelerating near the end
            double remainingDist = Math.abs(targetTicks) - avgPos;
            double decelThreshold = (MIN_POWER * 0.8 * MIN_POWER * 0.8 * MAX_VEL * MAX_VEL) / (2.0 * MAX_DECEL);

            if (remainingDist > decelThreshold) {
                // Not in final deceleration zone - enforce reduced minimum power
                double minVel = MIN_POWER * 0.8 * MAX_VEL;
                scaledVel = Math.max(scaledVel, minVel);
            }

            // Drift correction: compute error between left and right wheels
            int positionError = posL - posR;
            double correction = DRIFT_CORRECTION_KP * positionError;

            // Apply direction and drift correction
            double leftVel = direction * (scaledVel - correction);
            double rightVel = direction * (scaledVel + correction);

            leftWheel.setVelocity(leftVel);
            rightWheel.setVelocity(rightVel);

            opMode.idle();
        }

        stopDrive();
    }

    // -------------------------------------------------------------------
    // STOP DRIVE
    // -------------------------------------------------------------------
    private static void stopDrive() {
        leftWheel.setPower(0);
        rightWheel.setPower(0);
        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // -------------------------------------------------------------------
    // SHOOTER / GATEKEEPERS / INTAKE BASIC HELPERS
    // -------------------------------------------------------------------
    public static void setGatekeepers(double power) {
        leftGatekeeperServo.setPower(power * GATEKEEPER_LEFT_POWER);
        rightGatekeeperServo.setPower(power * GATEKEEPER_RIGHT_POWER);
    }

    public static void setIntake(double power) {
        intakeMotor.setPower(power);
    }

//    public static void TurnOnGatekeepersForXMilliSecondsAndTurnOff(int ms) {
//        setGatekeepers(1.0);
//        safeWait(ms);
//        setGatekeepers(0);
//    }
//
//    public static void TurnOnIntakeForXMilliSecondsAndTurnOff(int ms) {
//        intakeMotor.setPower(INTAKE_POWER);
//        safeWait(ms);
//        intakeMotor.setPower(0);
//    }
//
//    public static void TurnOnOutakeForXMilliSecondsAndTurnOff(int ms) {
//        if (!ALLOW_OUTTAKE) return;
//        intakeMotor.setPower(OUTTAKE_POWER);
//        safeWait(ms);
//        intakeMotor.setPower(0);
//    }

    // -------------------------------------------------------------------
    // TRAP — RESTORED WORKING VERSION (DO NOT "OPTIMIZE" YET)
    // -------------------------------------------------------------------
//    public static void OpenAndCloseTheTrapServo() {
//        trapServo.setPosition(TRAP_OPEN_POS);
//        safeWait(495); // your known-good
//        trapServo.setPosition(TRAP_CLOSED_POS);
//        safeWait(200); // reduced from 500; increase if needed
//    }

    // -------------------------------------------------------------------
    // SMART SHOOTING (USES TRAP AS-IS)
    // -------------------------------------------------------------------
    private static double shooterVFilt = 0;
    private static boolean shooterFilterInit = false;

    private static double updateShooterVFilt() {
        double v = shooter.getVelocity();
        if (!shooterFilterInit) {
            shooterVFilt = v;
            shooterFilterInit = true;
        } else {
            shooterVFilt = (SHOOTER_FILTER_ALPHA * v) + ((1.0 - SHOOTER_FILTER_ALPHA) * shooterVFilt);
        }
        return shooterVFilt;
    }

    private static void resetShooterFilterToCurrent() {
        shooterVFilt = shooter.getVelocity();
        shooterFilterInit = true;
    }

    public static boolean waitShooterReady(double targetVel) {
        final ElapsedTime stable = new ElapsedTime();
        stable.reset();
        resetShooterFilterToCurrent();

        return waitUntil(() -> {
            double raw  = shooter.getVelocity();
            double filt = updateShooterVFilt();

            boolean rawReady  = Math.abs(raw  - targetVel) <= SHOOT_READY_TOL;
            boolean filtReady = Math.abs(filt - targetVel) <= SHOOT_READY_TOL;

            if (rawReady && filtReady) {
                return stable.milliseconds() >= SHOOT_READY_STABLE_MS;
            } else {
                stable.reset();
                return false;
            }
        }, SHOOT_READY_TIMEOUT_MS);
    }


    private static boolean waitForShotEvent(double targetVel, long timeoutMs) {
        resetShooterFilterToCurrent();
        final ElapsedTime t = new ElapsedTime();
        t.reset();

        double peak = shooter.getVelocity();
        int dipCount = 0;

        while (activeOpMode != null && activeOpMode.opModeIsActive() && t.milliseconds() < timeoutMs) {
            double raw = shooter.getVelocity();
            if (raw > peak) peak = raw;

            boolean dipped = (peak - raw) > DIP_FROM_PEAK;

            if (dipped) dipCount++;
            else dipCount = 0;

            if (dipCount >= DIP_CONFIRM_COUNT && t.milliseconds() >= FEED_MIN_MS) return true;

            activeOpMode.idle();
        }
        return false;
    }

    public static void jamRecover() {
        setGatekeepers(0);

        intakeMotor.setPower(OUTTAKE_POWER);
        safeWait(JAM_REVERSE_MS);

        intakeMotor.setPower(INTAKE_POWER);
        safeWait(JAM_FORWARD_MS);

        intakeMotor.setPower(0);
    }

    public static boolean feedOneBallSmart(double shootTargetVel) {
        // Wait until shooter is stable before feeding
        waitShooterReady(shootTargetVel);

        double feedVel = shootTargetVel + FEED_OVERSPEED;
        shooter.setVelocity(feedVel);

        setIntake(INTAKE_POWER);
        setGatekeepers(1.0);

        boolean sawShot = waitForShotEvent(feedVel, FEED_MAX_MS);

        // Ensure ball clears
        safeWait(FEED_TAIL_MS);

        setGatekeepers(0);
        setIntake(0);

        shooter.setVelocity(shootTargetVel);

        // if (!sawShot) jamRecover();

        waitShooterReady(shootTargetVel);
        return sawShot;
    }

    /**
     * Smart 3-ball using your known-good trap timing.
     * Ball 1: feed
     * Ball 2: trap open/close (old reliable), then feed
     * Ball 3: trap open/close (old reliable), then feed
     */
    public static void shoot3PreloadsSmart(double shootTargetVel) {
        feedOneBallSmart(shootTargetVel);

        OpenAndCloseTheTrapServo();
        safeWait(TRAP_SEAT_MS);
        feedOneBallSmart(shootTargetVel);

        OpenAndCloseTheTrapServo();
        safeWait(TRAP_SEAT_MS);
        feedOneBallSmart(shootTargetVel);
    }

    public static void shoot3FinalSmart(double shootTargetVel) {
        feedOneBallSmart(shootTargetVel);

        OpenAndCloseTheTrapServo();
        safeWait(TRAP_SEAT_MS);
        feedOneBallSmart(shootTargetVel);

        OpenAndCloseTheTrapServo();
        safeWait(TRAP_SEAT_MS);
        feedOneBallSmart(shootTargetVel);
    }


    // -------------------------------------------------------------------
    // SHOOTER / GATEKEEPERS / INTAKE
    // -------------------------------------------------------------------
    public static void TurnOnGatekeepersForXMilliSecondsAndTurnOff(int ms) {
        leftGatekeeperServo.setPower(GATEKEEPER_LEFT_POWER);
        rightGatekeeperServo.setPower(GATEKEEPER_RIGHT_POWER);
        safeWait(ms);
        leftGatekeeperServo.setPower(0);
        rightGatekeeperServo.setPower(0);
    }

    public static void TurnOnIntakeForXMilliSecondsAndTurnOff(int ms) {
        intakeMotor.setPower(INTAKE_POWER);
        safeWait(ms);
        intakeMotor.setPower(0);
    }

    public static void TurnOnOutakeForXMilliSecondsAndTurnOff(int ms) {
        intakeMotor.setPower(OUTTAKE_POWER);
        safeWait(ms);
        intakeMotor.setPower(0);
    }

    public static void OpenAndCloseTheTrapServo() {
        trapServo.setPosition(TRAP_OPEN_POS);
        safeWait(495);
        trapServo.setPosition(TRAP_CLOSED_POS);
        safeWait(500);
    }

    // -------------------------------------------------------------------
    // TIME-BASED
    // -------------------------------------------------------------------
    public static void ThreeBallShootingProcess() {

        //first ball
        safeWait(400);

        TurnOnGatekeepersForXMilliSecondsAndTurnOff(500);
        safeWait(400);

        //second ball
        intakeMotor.setPower(-1);
        leftGatekeeperServo.setPower(1);
        rightGatekeeperServo.setPower(1);
        safeWait(1000);

        intakeMotor.setPower(0);
        leftGatekeeperServo.setPower(0);
        rightGatekeeperServo.setPower(0);
        safeWait(300);

        //3rd ball
        OpenAndCloseTheTrapServo();
        TurnOnOutakeForXMilliSecondsAndTurnOff(50);
        TurnOnIntakeForXMilliSecondsAndTurnOff(300);
        safeWait(200);
        TurnOnGatekeepersForXMilliSecondsAndTurnOff(500);
        safeWait(300);

        // Launch third ball again in case it failed last time
//        Robot.OpenAndCloseTheTrapServo();
        Robot.TurnOnIntakeForXMilliSecondsAndTurnOff(550);
        Robot.TurnOnGatekeepersForXMilliSecondsAndTurnOff(500);
    }

    public static void ThreeBallShootingProcess2() {

        //first ball
        safeWait(500);

        Robot.intakeMotor.setVelocity(Robot.INTAKE_VELOCITY);
        Robot.leftGatekeeperServo.setPower(1);
        Robot.rightGatekeeperServo.setPower(1);
        safeWait(3500);

    }

    public static void BlueFinalShoot() {
//
//        TurnOnGatekeepersForXMilliSecondsAndTurnOff(500);
//        safeWait(400);
        TurnOnOutakeForXMilliSecondsAndTurnOff(150);
        intakeMotor.setPower(-1);
        leftGatekeeperServo.setPower(1);
        rightGatekeeperServo.setPower(1);
        safeWait(850);

        intakeMotor.setPower(0);
        leftGatekeeperServo.setPower(0);
        rightGatekeeperServo.setPower(0);
        safeWait(450);

        OpenAndCloseTheTrapServo();
        TurnOnOutakeForXMilliSecondsAndTurnOff(50);
        TurnOnIntakeForXMilliSecondsAndTurnOff(300);
        safeWait(200);
        TurnOnGatekeepersForXMilliSecondsAndTurnOff(500);
        safeWait(300);

        OpenAndCloseTheTrapServo();
        TurnOnOutakeForXMilliSecondsAndTurnOff(50);
        TurnOnIntakeForXMilliSecondsAndTurnOff(300);
        safeWait(200);
        TurnOnGatekeepersForXMilliSecondsAndTurnOff(500);
        safeWait(300);
        // Launch third ball again in case it failed last time
        Robot.OpenAndCloseTheTrapServo();
        Robot.TurnOnIntakeForXMilliSecondsAndTurnOff(550);
        Robot.TurnOnGatekeepersForXMilliSecondsAndTurnOff(500);
    }
    public static void RedFinalShoot() {
//
//        TurnOnGatekeepersForXMilliSecondsAndTurnOff(500);
//        safeWait(400);
        TurnOnOutakeForXMilliSecondsAndTurnOff(120);
        intakeMotor.setPower(-1);
        leftGatekeeperServo.setPower(1);
        rightGatekeeperServo.setPower(1);
        safeWait(850);

        intakeMotor.setPower(0);
        leftGatekeeperServo.setPower(0);
        rightGatekeeperServo.setPower(0);
        safeWait(450);

        OpenAndCloseTheTrapServo();
        TurnOnOutakeForXMilliSecondsAndTurnOff(50);
        TurnOnIntakeForXMilliSecondsAndTurnOff(300);
        safeWait(200);
        TurnOnGatekeepersForXMilliSecondsAndTurnOff(500);
        safeWait(300);

        OpenAndCloseTheTrapServo();
        TurnOnOutakeForXMilliSecondsAndTurnOff(50);
        TurnOnIntakeForXMilliSecondsAndTurnOff(300);
        safeWait(200);
        TurnOnGatekeepersForXMilliSecondsAndTurnOff(500);
        safeWait(300);
        // Launch third ball again in case it failed last time
        Robot.OpenAndCloseTheTrapServo();
        Robot.TurnOnIntakeForXMilliSecondsAndTurnOff(550);
        Robot.TurnOnGatekeepersForXMilliSecondsAndTurnOff(500);
    }


    public static void TestShoot2() {

        TurnOnGatekeepersForXMilliSecondsAndTurnOff(500);
        safeWait(400);

        intakeMotor.setPower(-1);
        leftGatekeeperServo.setPower(1);
        rightGatekeeperServo.setPower(1);
        safeWait(1000);

        intakeMotor.setPower(0);
        leftGatekeeperServo.setPower(0);
        rightGatekeeperServo.setPower(0);
        safeWait(300);

        OpenAndCloseTheTrapServo();
        TurnOnOutakeForXMilliSecondsAndTurnOff(50);
        TurnOnIntakeForXMilliSecondsAndTurnOff(300);
        safeWait(200);
        TurnOnGatekeepersForXMilliSecondsAndTurnOff(500);
        safeWait(300);
        // Launch third ball again in case it failed last time
        Robot.OpenAndCloseTheTrapServo();
        Robot.TurnOnIntakeForXMilliSecondsAndTurnOff(550);
        Robot.TurnOnGatekeepersForXMilliSecondsAndTurnOff(500);
    }

    //League meet 1 method
    public static void DriveTrainBackwardForXMilliSecondsAndTurnOff(int milliseconds) {
        rightWheel.setPower(-1);
        leftWheel.setPower(-1);
        safeWait(milliseconds);
        rightWheel.setPower(0);
        leftWheel.setPower(0);

    }
    public static void GoingForward(int ms, double power) {
        leftWheel.setPower(power);
        rightWheel.setPower(power);
        safeWait(ms);
        stopDrive();
    }

    public static void GoingBackward(int ms, double power) {
        leftWheel.setPower(-power);
        rightWheel.setPower(-power);
        safeWait(ms);
        stopDrive();
    }
}