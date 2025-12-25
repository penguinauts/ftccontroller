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
    public static double TRAP_OPEN_POS   = 0.77;
    public static double TRAP_CLOSED_POS = 0.55;

    public static double INTAKE_POWER  = -1.0;
    public static double INTAKE_VELOCITY = -2500;
    //-4000 is original number
    public static double OUTTAKE_POWER = 1.0;

    public static double GATEKEEPER_LEFT_POWER = 1.0;
    public static double GATEKEEPER_RIGHT_POWER = 1.0;

    //public static double GATEKEEPER_STOP_POWER    = 0.0;

    // -------------------------------------------------------------------
    // SHOOTER PIDF (TUNABLE)
    // -------------------------------------------------------------------
    // Recommended PIDF tuning for better response to load changes:
    // - Increase P if recovery is too slow
    // - Increase D to reduce overshoot/oscillation
    // - Increase F if steady-state velocity is below target
    public static int PROPORTIONAL = 350;
    public static int INTEGRAL     = 0;
    public static int DERIVATIVE   = 10;
    public static int FEED_FORWARD = 14;

    public static PIDFCoefficients SHOOTER_PIDF =
            new PIDFCoefficients(PROPORTIONAL, INTEGRAL, DERIVATIVE, FEED_FORWARD);

    // -------------------------------------------------------------------
    // SHOOTER VELOCITY MANAGEMENT
    // -------------------------------------------------------------------
    public static double SHOOTER_VELOCITY_TOLERANCE = 50;  // ticks/sec tolerance
    public static int SHOOTER_RECOVERY_TIME_MS = 300;      // minimum wait after each shot

    /**
     * Updates SHOOTER_PIDF from the tunable constants.
     * Call this after changing P/I/D/F values via dashboard.
     */
    public static void updateShooterPIDF() {
        SHOOTER_PIDF = new PIDFCoefficients(PROPORTIONAL, INTEGRAL, DERIVATIVE, FEED_FORWARD);
        if (shooter != null) {
            shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, SHOOTER_PIDF);
        }
    }

    // -------------------------------------------------------------------
    // ENCODER VALUES
    // -------------------------------------------------------------------
    public static double WHEEL_DIAMETER_INCHES = 3.78;
    //public static double TICKS_PER_ROTATION   = 537.7;
    public static double TICKS_PER_INCH       = 52.2;
    public static double WHEEL_CIRCUMFERENCE  = Math.PI * WHEEL_DIAMETER_INCHES;

    //public static double INCHES_PER_90_DEG = 18.0;

    public static double TURN_ROTATION_P = 150;

    public static void recomputeConstants() {
        WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_INCHES;
    }

    // -------------------------------------------------------------------
    // INITIALIZATION (NO IMU)
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
        //shooter.setDirection(DcMotorSimple.Direction.FORWARD); //remove this later

        // Intake
        intakeMotor = hw.get(DcMotorEx.class, "Intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        //intakeMotor.setDirection(DcMotor.Direction.FORWARD);
//        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  //old
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);



        // Drive wheels
        leftWheel  = hw.get(DcMotorEx.class, "leftWheel");
        rightWheel = hw.get(DcMotorEx.class, "rightWheel");

        imu = hw.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));  //on the other control hub
        //imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));


        leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //leftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightWheel.setDirection(DcMotor.Direction.FORWARD);

        // Reset encoders
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Calculates the target velocity at a given position using a motion profile.
     * Supports both trapezoidal (full speed reached) and triangular (peak speed limited) profiles.
     *
     * @param targetTicks Total distance to travel (in ticks)
     * @param currentTicks Current position (in ticks)
     * @return Target velocity at current position (in ticks/sec)
     */
    public static double getProfiledVelocity(double targetTicks, double currentTicks) {
        // Input validation
        if (targetTicks == 0) return 0;

        // Work with absolute values for simplicity
        double x = Math.abs(currentTicks);
        double total = Math.abs(targetTicks);

        // Clamp current position to valid range
        if (x > total) {
            x = total;
        }

        // Distance required to accelerate/decelerate to/from max velocity
        double accelDist = (MAX_VEL * MAX_VEL) / (2.0 * MAX_ACCEL);
        double decelDist = (MAX_VEL * MAX_VEL) / (2.0 * MAX_DECEL);

        double vel;

        // Check if robot can reach full speed
        if (accelDist + decelDist > total) {
            // TRIANGULAR PROFILE: Cannot reach MAX_VEL
            // Calculate peak velocity and transition point
            double peakVel = Math.sqrt(2.0 * MAX_ACCEL * MAX_DECEL * total / (MAX_ACCEL + MAX_DECEL));
            double transitionPoint = (peakVel * peakVel) / (2.0 * MAX_ACCEL);

            if (x < transitionPoint) {
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
    // SAFE WAIT
    // -------------------------------------------------------------------
    public static void safeWait(long ms) {
        if (activeOpMode == null) {
            try { Thread.sleep(ms); } catch (InterruptedException ignored) {}
            return;
        }

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (activeOpMode.opModeIsActive() &&
                timer.milliseconds() < ms) {
            activeOpMode.idle();
        }
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
        double estimatedTime = (Math.abs(targetTicks) / MAX_VEL) * 1.5 + 2.0; // 50% margin + 2sec
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
            double avgPos = (posL + posR) / 2.0; // Use double for precision

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
        } while ((Math.abs(error) > tolerance || Math.abs(imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate) > velocityTolerance) && opMode.opModeIsActive());
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
            double avgPos = (posL + posR) / 2.0; // Use double for precision

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
    // SHOOTER / GATEKEEPERS / INTAKE
    // -------------------------------------------------------------------
    /**
     * Waits for the shooter motor to reach the target velocity within tolerance.
     * Prevents feeding balls before shooter is ready.
     *
     * @param targetVelocity Target velocity in ticks/sec
     * @param timeoutMs Maximum time to wait in milliseconds
     * @return true if target velocity reached, false if timeout
     */
    public static boolean waitForShooterVelocity(double targetVelocity, long timeoutMs) {
        if (activeOpMode == null) return true;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (activeOpMode.opModeIsActive() && timer.milliseconds() < timeoutMs) {
            double currentVel = shooter.getVelocity();
            double error = Math.abs(targetVelocity - currentVel);

            // Optional telemetry for debugging
            if (activeOpMode.telemetry != null) {
                activeOpMode.telemetry.addData("Shooter Target", "%.0f", targetVelocity);
                activeOpMode.telemetry.addData("Shooter Current", "%.0f", currentVel);
                activeOpMode.telemetry.addData("Shooter Error", "%.0f", error);
                activeOpMode.telemetry.update();
            }

            if (error <= SHOOTER_VELOCITY_TOLERANCE) {
                return true; // Within tolerance
            }

            activeOpMode.idle();
        }

        return false; // Timeout
    }

    /**
     * Sets shooter velocity and waits for it to stabilize.
     * Use this instead of directly calling setVelocity when precision matters.
     *
     * @param targetVelocity Target velocity in ticks/sec
     */
    public static void setShooterVelocityAndWait(double targetVelocity) {
        shooter.setVelocity(targetVelocity);
        waitForShooterVelocity(targetVelocity, 2000); // 2 second timeout
    }

    /**
     * Verifies shooter has recovered velocity after shooting a ball.
     * Should be called between each shot for consistency.
     *
     * @param targetVelocity Target velocity that shooter should maintain
     * @return true if velocity recovered, false if not
     */
    public static boolean verifyShooterRecovery(double targetVelocity) {
        // Short wait to let motor respond to load
        safeWait(SHOOTER_RECOVERY_TIME_MS);

        // Check if velocity has recovered
        double currentVel = shooter.getVelocity();
        double error = Math.abs(targetVelocity - currentVel);

        return error <= SHOOTER_VELOCITY_TOLERANCE;
    }

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
    // TIME-BASED SHOOTING SEQUENCES
    // -------------------------------------------------------------------
    /**
     * Improved three-ball shooting process with velocity monitoring.
     * Ensures shooter maintains consistent velocity between shots.
     *
     * @param targetShooterVelocity The velocity the shooter should maintain
     */
    public static void ThreeBallShootingProcess(double targetShooterVelocity) {

        // ===== FIRST BALL =====
        // Ensure shooter is at speed before shooting
        waitForShooterVelocity(targetShooterVelocity, 1500);
        safeWait(200);  // Extra stability time

        TurnOnGatekeepersForXMilliSecondsAndTurnOff(500);

        // Wait for shooter to recover after first ball
        if (!verifyShooterRecovery(targetShooterVelocity)) {
            safeWait(200);  // Extra time if not recovered
        }

        // ===== SECOND BALL =====
        intakeMotor.setPower(-1);
        leftGatekeeperServo.setPower(1);
        rightGatekeeperServo.setPower(1);
        safeWait(1000);

        intakeMotor.setPower(0);
        leftGatekeeperServo.setPower(0);
        rightGatekeeperServo.setPower(0);

        // Wait for shooter to recover after second ball
        if (!verifyShooterRecovery(targetShooterVelocity)) {
            safeWait(200);  // Extra time if not recovered
        }

        // ===== THIRD BALL =====
        OpenAndCloseTheTrapServo();
        TurnOnOutakeForXMilliSecondsAndTurnOff(50);
        TurnOnIntakeForXMilliSecondsAndTurnOff(300);
        safeWait(200);

        // Ensure shooter is ready for third ball
        waitForShooterVelocity(targetShooterVelocity, 1000);

        TurnOnGatekeepersForXMilliSecondsAndTurnOff(500);
        safeWait(300);

        // Retry third ball if needed
        TurnOnIntakeForXMilliSecondsAndTurnOff(550);
        verifyShooterRecovery(targetShooterVelocity);
        TurnOnGatekeepersForXMilliSecondsAndTurnOff(500);
    }

    /**
     * Original three-ball shooting (no velocity parameter for backward compatibility).
     * Uses the method with default behavior.
     */
    public static void ThreeBallShootingProcess() {
        // Get current shooter velocity as target
        double currentVelocity = shooter.getVelocity();
        ThreeBallShootingProcess(Math.abs(currentVelocity));
    }

    public static void ThreeBallShootingProcess2() {

        //first ball
        safeWait(500);

        Robot.intakeMotor.setVelocity(Robot.INTAKE_VELOCITY);
        Robot.leftGatekeeperServo.setPower(1);
        Robot.rightGatekeeperServo.setPower(1);
        safeWait(3500);


//        //second ball
//        intakeMotor.setPower(-1);
//        leftGatekeeperServo.setPower(1);
//        rightGatekeeperServo.setPower(1);
//        safeWait(1000);
//
//        intakeMotor.setPower(0);
//        leftGatekeeperServo.setPower(0);
//        rightGatekeeperServo.setPower(0);
//        safeWait(300);

//        //3rd ball
//        OpenAndCloseTheTrapServo();
//        TurnOnOutakeForXMilliSecondsAndTurnOff(50);
//        TurnOnIntakeForXMilliSecondsAndTurnOff(300);
//        safeWait(200);
//        TurnOnGatekeepersForXMilliSecondsAndTurnOff(500);
//        safeWait(300);
//
//        // Launch third ball again in case it failed last time
////        Robot.OpenAndCloseTheTrapServo();
//        Robot.TurnOnIntakeForXMilliSecondsAndTurnOff(550);
//        Robot.TurnOnGatekeepersForXMilliSecondsAndTurnOff(500);
    }

    /**
     * Improved test shooting sequence with velocity monitoring.
     * Used for the second set of 3 balls after intake.
     *
     * @param targetShooterVelocity The velocity the shooter should maintain
     */
    public static void TestShoot1(double targetShooterVelocity) {
        // Ensure shooter is at full speed before starting
        waitForShooterVelocity(targetShooterVelocity, 1500);

        // ===== FIRST BALL (already loaded) =====
        intakeMotor.setPower(-1);
        leftGatekeeperServo.setPower(1);
        rightGatekeeperServo.setPower(1);
        safeWait(1000);

        intakeMotor.setPower(0);
        leftGatekeeperServo.setPower(0);
        rightGatekeeperServo.setPower(0);

        // Wait for shooter to recover
        verifyShooterRecovery(targetShooterVelocity);

        // ===== SECOND BALL =====
        OpenAndCloseTheTrapServo();
        TurnOnOutakeForXMilliSecondsAndTurnOff(50);
        TurnOnIntakeForXMilliSecondsAndTurnOff(300);
        safeWait(200);

        // Ensure shooter is ready
        waitForShooterVelocity(targetShooterVelocity, 1000);

        TurnOnGatekeepersForXMilliSecondsAndTurnOff(500);

        // Wait for shooter to recover
        verifyShooterRecovery(targetShooterVelocity);

        // ===== THIRD BALL =====
        OpenAndCloseTheTrapServo();
        TurnOnOutakeForXMilliSecondsAndTurnOff(50);
        TurnOnIntakeForXMilliSecondsAndTurnOff(300);
        safeWait(200);

        // Ensure shooter is ready
        waitForShooterVelocity(targetShooterVelocity, 1000);

        TurnOnGatekeepersForXMilliSecondsAndTurnOff(500);

        // Retry third ball if needed
        safeWait(300);
        OpenAndCloseTheTrapServo();
        TurnOnIntakeForXMilliSecondsAndTurnOff(550);
        verifyShooterRecovery(targetShooterVelocity);
        TurnOnGatekeepersForXMilliSecondsAndTurnOff(500);
    }

    /**
     * Original TestShoot1 (no velocity parameter for backward compatibility).
     */
    public static void TestShoot1() {
        double currentVelocity = shooter.getVelocity();
        TestShoot1(Math.abs(currentVelocity));
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

    // -------------------------------------------------------------------
    // Codes not being used at the moment
    // -------------------------------------------------------------------
//    public static void driveStraightInches(LinearOpMode opMode,
//                                           double inches,
//                                           double maxPower,
//                                           long waitMs) {
//
//        // Call the original movement function
//        driveStraightInches(opMode, inches, maxPower);
//
//        // Then wait
//        safeWait(waitMs);
//    }
    //No IMU
//    public static void turnDegrees(LinearOpMode opMode,
//                                   double degrees,
//                                   double maxPower,
//                                   double inchesPer90Deg) {
//
//        recomputeConstants();
//
//        // Convert degrees to equivalent linear wheel travel
//        double inches = (degrees / 90.0) * inchesPer90Deg;
//        int targetTicks = (int)(inches * TICKS_PER_INCH);
//        double direction = Math.signum(degrees);
//
//        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        while (opMode.opModeIsActive()) {
//
//            int posL = Math.abs(leftWheel.getCurrentPosition());
//            int posR = Math.abs(rightWheel.getCurrentPosition());
//            int avg = (posL + posR) / 2;
//
//            if (avg >= Math.abs(targetTicks)) break;
//
//            double targetVel = getProfiledVelocity(targetTicks, avg);
//
//            // Smooth scaling
//            double scaledPower = Math.max(MIN_POWER, maxPower * (targetVel / MAX_VEL));
//
//            // Turning often needs slightly more startup torque
//            if (scaledPower < 0.18) {
//                scaledPower = 0.18;
//            }
//
//            double scaledVel = MAX_VEL * scaledPower;
//
//            // left and right wheels must spin opposite directions for turning
//            leftWheel.setVelocity(direction * scaledVel);
//            rightWheel.setVelocity(-direction * scaledVel);
//
//            opMode.idle();
//        }
//
//        stopDrive();
//    }
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
//    public static void ThreeBallShootingProcess() {
//
//        TurnOnGatekeepersForXMilliSecondsAndTurnOff(1000);
//        safeWait(400);
//
//        intakeMotor.setPower(-1);
//        leftGatekeeperServo.setPower(1);
//        rightGatekeeperServo.setPower(1);
//        safeWait(1000);
//
//        intakeMotor.setPower(0);
//        leftGatekeeperServo.setPower(0);
//        rightGatekeeperServo.setPower(0);
//        safeWait(400);
//
//        OpenAndCloseTheTrapServo();
//        TurnOnOutakeForXMilliSecondsAndTurnOff(50);
//        TurnOnIntakeForXMilliSecondsAndTurnOff(300);
//        safeWait(200);
//        TurnOnGatekeepersForXMilliSecondsAndTurnOff(1000);
//        safeWait(500);
//    }


}