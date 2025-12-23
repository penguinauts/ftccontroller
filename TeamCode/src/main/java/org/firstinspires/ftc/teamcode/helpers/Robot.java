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
    public static int PROPORTIONAL = 350;
    public static int INTEGRAL     = 0;
    public static int DERIVATIVE   = 10;
    public static int FEED_FORWARD = 14;

    public static PIDFCoefficients SHOOTER_PIDF =
            new PIDFCoefficients(PROPORTIONAL, INTEGRAL, DERIVATIVE, FEED_FORWARD);

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

    public static double getProfiledVelocity(double targetTicks, double currentTicks) {

        double x = Math.abs(currentTicks);
        double total = Math.abs(targetTicks);

        // Distance required to accelerate to max velocity
        double accelDist = (MAX_VEL * MAX_VEL) / (2.0 * MAX_ACCEL);
        double decelDist = (MAX_VEL * MAX_VEL) / (2.0 * MAX_DECEL);

        double vel;

        // If robot cannot reach full speed → triangular profile
        if (accelDist + decelDist > total) {
//        if (2 * accelDist > total) {
            if (x < accelDist) {
//            if (x < total / 2.0) {
                vel = Math.sqrt(2.0 * MAX_ACCEL * x);
            } else {
                double remain = total - x;
                vel = Math.sqrt(2.0 * MAX_DECEL * remain);
//                vel = Math.sqrt(2.0 * MAX_ACCEL * remain);
            }
        }
        else {
            // Trapezoidal profile
            if (x < accelDist) {
                vel = Math.sqrt(2.0 * MAX_ACCEL * x);
            }
            else if (x > (total - decelDist)) {
//            else if (x > (total - accelDist)) {
                double remain = total - x;
                vel = Math.sqrt(2.0 * MAX_DECEL * remain);
//                vel = Math.sqrt(2.0 * MAX_ACCEL * remain);
            }
            else {
                vel = MAX_VEL;
            }
        }

        return vel;
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
    public static void driveStraightInches(LinearOpMode opMode,
                                           double inches,
                                           double maxPower) {

        recomputeConstants();

        int targetTicks = (int)(inches * TICKS_PER_INCH);
        double direction = Math.signum(inches);

        // Reset encoders
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opMode.opModeIsActive()) {

            int posL = Math.abs(leftWheel.getCurrentPosition());
            int posR = Math.abs(rightWheel.getCurrentPosition());
            int avg = (posL + posR) / 2;

            // Stop when reached target distance
            if (avg >= Math.abs(targetTicks)) break;

            // Desired velocity at this point in the motion profile
            double targetVel = getProfiledVelocity(targetTicks, avg);

            // Scale targetVel by maxPower but ensure MIN_POWER is enforced
            double scaledPower = Math.max(MIN_POWER, maxPower * (targetVel / MAX_VEL));

            // Convert scaled power back into a velocity command
            double scaledVel = MAX_VEL * scaledPower;

            // Apply direction (forward or backward)
            leftWheel.setVelocity(direction * scaledVel);
            rightWheel.setVelocity(direction * scaledVel);

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
    public static void driveStraightSlowInches(LinearOpMode opMode,
                                               double inches,
                                               double maxPower) {

        recomputeConstants();

        int targetTicks = (int)(inches * TICKS_PER_INCH);
        double direction = Math.signum(inches);

        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opMode.opModeIsActive()) {

            int posL = Math.abs(leftWheel.getCurrentPosition());
            int posR = Math.abs(rightWheel.getCurrentPosition());
            int avg = (posL + posR) / 2;

            if (avg >= Math.abs(targetTicks)) break;

            // base profiled velocity
            double targetVel = getProfiledVelocity(targetTicks, avg);

            // slow mode: clamp velocity down with maxPower
            double scaledPower = maxPower * (targetVel / MAX_VEL);

            // ensure slow drive still breaks friction
            scaledPower = Math.max(scaledPower, MIN_POWER * 0.8);

            double scaledVel = MAX_VEL * scaledPower;

            leftWheel.setVelocity(direction * scaledVel);
            rightWheel.setVelocity(direction * scaledVel);

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

    public static void TestShoot1() {

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