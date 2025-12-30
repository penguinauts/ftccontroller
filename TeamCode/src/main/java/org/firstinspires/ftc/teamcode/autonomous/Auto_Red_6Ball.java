package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.helpers.Robot;

@Config
@Autonomous
public class Auto_Red_6Ball extends LinearOpMode {

    // -----------------------------
    // SHOOTER TUNING
    // -----------------------------
    public static double SHOOTER_FIRE_VELOCITY  = 1050;
    public static double SHOOTER_INTAKE_VELOCITY = -200;

    // -----------------------------
    // DISTANCES (DASHBOARD TUNABLE)
    // -----------------------------
    public static double BACK_UP_FROM_START_INCHES    = 41;
    public static double FORWARD_AFTER_TURN_INCHES    = 9;
    public static double SLOW_FORWARD_INTAKE_INCHES   = 35.0;
    public static double BACK_TO_RAMP_INCHES          = 28.5;
    public static double FINAL_FORWARD_TO_RAMP_INCHES = 35.0;

    // -----------------------------
    // TURN ANGLES (TUNABLE)
    // -----------------------------
    public static double TURN_TO_INTAKE = 47;
    public static double TURN_TO_SHOOT = -48.0;
    //public static double TURN_3_DEGREES = 0.0;

    // -----------------------------
    // DRIVE POWERS (NOW TUNABLE)
    // -----------------------------
    //public static double DRIVE_POWER_BACKUP        = 1.0;
    //public static double DRIVE_POWER_FORWARD_TURN  = 1.0;
    public static double DRIVE_POWER_SLOW_INTAKE   = 0.40;
    //public static double DRIVE_POWER_RETURN        = 1.0;
    //public static double DRIVE_POWER_FINAL_FORWARD = 1.0;

    //public static double DRIVE_POWER_TURN_1 = 0.6;
    //public static double DRIVE_POWER_TURN_2 = 0.6;
    //public static double DRIVE_POWER_TURN_3 = 0.5;
    public static double FINAL_EXIT_1 = -5.0;
    public static double FINAL_EXIT_2 = -22.0;
    public static double FINAL_TURN = -60.0;
    public static double GATEKEEPER_POWER = -1;


    @Override
    public void runOpMode() {

        Robot.initializeRobot(hardwareMap);
        Robot.recomputeConstants();
        Robot.activeOpMode = this;  // required for safeWait()

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Auto Blue - 6 Ball READY");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;


//         ============================================================
//         STEP 1 — SPIN SHOOTER FOR PRELOADS
//         ============================================================
        Robot.shooter.setVelocity(SHOOTER_FIRE_VELOCITY);
        Robot.safeWait(750);
//
//        // ============================================================
//        // STEP 2 — SHOOT FIRST 3 BALLS
//        // ============================================================
        Robot.ThreeBallShootingProcess();
//
//         ============================================================
//         STEP 3 — DRIVE TO FIELD BALLS
//         ============================================================
        Robot.driveStraightInches(this, -BACK_UP_FROM_START_INCHES, 1.0);
        Robot.safeWait(200);

        // Robot.turnDegrees(this, -TURN_1_DEGREES, DRIVE_POWER_TURN_1, Robot.INCHES_PER_90_DEG);
        Robot.turnDegreesIMU(this, TURN_TO_INTAKE);
        Robot.intakeMotor.setPower(Robot.INTAKE_POWER);
        Robot.safeWait(50);

        // ============================================================
        // STEP 4 — SHOOTER REVERSE WHILE INTAKING
        // ============================================================
        Robot.shooter.setVelocity(SHOOTER_INTAKE_VELOCITY);
//        Robot.intakeMotor.setPower(Robot.INTAKE_POWER);
        Robot.leftGatekeeperServo.setPower(GATEKEEPER_POWER);
        Robot.rightGatekeeperServo.setPower(GATEKEEPER_POWER);
        Robot.trapServo.setPosition(Robot.TRAP_OPEN_POS);

//        Robot.driveStraightInches(this, FORWARD_AFTER_TURN_INCHES, 1.0);
//        Robot.safeWait(100);

        Robot.driveStraightSlowInches(this, SLOW_FORWARD_INTAKE_INCHES, DRIVE_POWER_SLOW_INTAKE);

        // ============================================================
        // STEP 5 — RETURN TO SHOOTING POSITION
        // ===========================================================

        Robot.intakeMotor.setPower(-0.5);
        Robot.driveStraightInches(this, -BACK_TO_RAMP_INCHES, 1.0);
        //        Robot.intakeMotor.setPower(0);
        Robot.trapServo.setPosition(Robot.TRAP_CLOSED_POS);
        Robot.leftGatekeeperServo.setPower(0);
        Robot.rightGatekeeperServo.setPower(0);
        //Robot.safeWait(100);
        Robot.intakeMotor.setPower(0);
        Robot.safeWait(200);

//        Robot.turnDegrees(this, TURN_2_DEGREES, DRIVE_POWER_TURN_2, Robot.INCHES_PER_90_DEG);
        Robot.turnDegreesIMU(this,TURN_TO_SHOOT);
        Robot.safeWait(200);
        Robot.shooter.setVelocity(SHOOTER_FIRE_VELOCITY);
        Robot.driveStraightInches(this, FINAL_FORWARD_TO_RAMP_INCHES, 1.0);
//        Robot.safeWait(200);


//        Robot.turnDegrees(this, TURN_3_DEGREES, DRIVE_POWER_TURN_3, Robot.INCHES_PER_90_DEG);

        // ============================================================
        // STEP 6 — SPIN SHOOTER FORWARD AGAIN FOR FINAL 3
        // ============================================================
//        Robot.shooter.setVelocity(SHOOTER_FIRE_VELOCITY + 50);
//        Robot.safeWait(600);

//        Robot.trapServo.setPosition(Robot.TRAP_OPEN_POS);
        Robot.safeWait(750);

        // ============================================================
        // STEP 7 — SHOOT LAST 3 BALLS
        // ============================================================
        Robot.TestShoot1();

        // ============================================================
        // STEP 8 — PARK
        // ============================================================
        Robot.driveStraightInches(this, FINAL_EXIT_1, 1.0);
        Robot.turnDegreesIMU(this, FINAL_TURN);
        Robot.safeWait(50);
        Robot.driveStraightInches(this, FINAL_EXIT_2, 1.0);

//        // ----------------------------------------------------
//        // TELEMETRY
//        // ----------------------------------------------------
//        telemetry.addData("ShooterVelocity", Robot.shooter.getVelocity());
//        telemetry.addData("RightWheelVelocity", Robot.rightWheel.getVelocity());
//        telemetry.addData("LeftWheelVelocity", Robot.leftWheel.getVelocity());
//        telemetry.addData("IntakePower", Robot.intakeMotor.getPower());
//        telemetry.addData("ShooterCurrent(mA)", Robot.shooter.getCurrent(CurrentUnit.MILLIAMPS));
//        telemetry.update();
    }
}