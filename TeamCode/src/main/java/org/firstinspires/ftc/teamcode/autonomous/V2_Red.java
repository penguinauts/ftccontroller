package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.helpers.Robot;

@Config
@Autonomous(name="V2 Red", group="Red Side")
public class V2_Red extends LinearOpMode {

    // -----------------------------
    // SHOOTER TUNING
    // -----------------------------
    public static double SHOOTER_FIRE_VELOCITY  = 1183;
    public static double SHOOTER_INTAKE_VELOCITY = -200;

    // -----------------------------
    // DISTANCES (TUNABLE)
    // -----------------------------
    public static double BACK_UP_FROM_START_INCHES    = 46.0;
    public static double FORWARD_AFTER_TURN_INCHES    = 28.0;
    public static double SLOW_FORWARD_INTAKE_INCHES   = 8.0;
    public static double BACK_TO_RAMP_INCHES          = 21.0;
    public static double FINAL_FORWARD_TO_RAMP_INCHES = 32.0;

    public static double FINAL_EXIT_1 = -5.0;
    public static double FINAL_EXIT_2 = -30.0;
    public static double FINAL_TURN = -40.0;
    public static double GATEKEEPER_POWER = -1;





    // -----------------------------
    // TURN ANGLES (MIRRORED FOR RED)
    // -----------------------------
    public static double TURN_TO_INTAKE =  48.0;   // was +46 for Blue
    public static double TURN_TO_SHOOT = -53.0;   // was +51 for Blue
    //public static double TURN_3_DEGREES = 0.0;

    // -----------------------------
    // DRIVE POWERS
    // -----------------------------
    //public static double DRIVE_POWER_BACKUP        = 1.0;
    //public static double DRIVE_POWER_FORWARD_TURN  = 1.0;
    public static double DRIVE_POWER_SLOW_INTAKE   = 0.50;
    //public static double DRIVE_POWER_RETURN        = 1.0;
    //public static double DRIVE_POWER_FINAL_FORWARD = 1.0;

    //public static double DRIVE_POWER_TURN_1 = 0.6;
    //public static double DRIVE_POWER_TURN_2 = 0.6;
    //public static double DRIVE_POWER_TURN_3 = 0.5;

    @Override
    public void runOpMode() {

        Robot.initializeRobot(hardwareMap);
        Robot.recomputeConstants();
        Robot.activeOpMode = this;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Auto Red - 6 Ball READY");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        // ============================================================
        // STEP 1 — SPIN SHOOTER FOR PRELOADS
        // ============================================================
        Robot.shooter.setVelocity(SHOOTER_FIRE_VELOCITY);
        Robot.safeWait(750);

        // ============================================================
        // STEP 2 — SHOOT FIRST 3 BALLS
        // ============================================================
        Robot.ThreeBallShootingProcess();

        // ============================================================
        // STEP 3 — DRIVE TO FIELD BALLS
        // ============================================================
        Robot.driveStraightInches(this, -BACK_UP_FROM_START_INCHES, 1.0);
        Robot.safeWait(200);

        // Mirrored turn direction for Red
        Robot.turnDegreesIMU(this, TURN_TO_INTAKE);
        Robot.safeWait(50);

        // ============================================================
        // STEP 4 — SHOOTER REVERSE WHILE INTAKING
        // ============================================================
        Robot.shooter.setVelocity(SHOOTER_INTAKE_VELOCITY);

        Robot.driveStraightInches(this, FORWARD_AFTER_TURN_INCHES, 1.0);
        Robot.safeWait(100);

        Robot.intakeMotor.setPower(Robot.INTAKE_POWER);
        Robot.leftGatekeeperServo.setPower(GATEKEEPER_POWER);
        Robot.rightGatekeeperServo.setPower(GATEKEEPER_POWER);
        Robot.trapServo.setPosition(Robot.TRAP_OPEN_POS);


        Robot.driveStraightSlowInches(this, SLOW_FORWARD_INTAKE_INCHES, DRIVE_POWER_SLOW_INTAKE);

//        Robot.intakeMotor.setPower(0);
        Robot.leftGatekeeperServo.setPower(0);
        Robot.rightGatekeeperServo.setPower(0);
        Robot.trapServo.setPosition(Robot.TRAP_CLOSED_POS);
        Robot.safeWait(100);

        // ============================================================
        // STEP 5 — RETURN TO SHOOTING POSITION
        // ============================================================
        Robot.driveStraightInches(this, -BACK_TO_RAMP_INCHES, 1.0);
        Robot.intakeMotor.setPower(0);
        Robot.safeWait(200);

//        Robot.turnDegrees(this, TURN_2_DEGREES, DRIVE_POWER_TURN_2, Robot.INCHES_PER_90_DEG);
        Robot.turnDegreesIMU(this,TURN_TO_SHOOT);
        Robot.safeWait(200);
        Robot.shooter.setVelocity(SHOOTER_FIRE_VELOCITY);
        Robot.driveStraightInches(this, FINAL_FORWARD_TO_RAMP_INCHES, 1.0);
        Robot.safeWait(200);


        // ============================================================
        // STEP 6 — SPIN SHOOTER FORWARD FOR FINAL 3
        // ============================================================
//        Robot.shooter.setVelocity(SHOOTER_FIRE_VELOCITY);
//        Robot.safeWait(600);

        Robot.trapServo.setPosition(Robot.TRAP_OPEN_POS);
        Robot.safeWait(150);

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
    }
}