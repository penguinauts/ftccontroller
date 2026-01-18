package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.helpers.Robot;

@Config
@Autonomous(name="Blue Ball Auto", group="Blue Side")
public class Auto_Blue_V1_6Ball extends LinearOpMode {

    // -----------------------------
    // SHOOTER TUNING
    // -----------------------------
    public static double SHOOTER_FIRE_VELOCITY  = 1100;
    public static double SHOOTER_INTAKE_VELOCITY = -200;

    // -----------------------------
    // DISTANCES (DASHBOARD TUNABLE)
    // -----------------------------
    public static double BACK_UP_FROM_START_INCHES    = 34.0;
    public static double FORWARD_AFTER_TURN_INCHES    = 11;
    public static double SLOW_FORWARD_INTAKE_INCHES   = 18.0;
    public static double BACK_TO_RAMP_INCHES          = 32.0;
    public static double FINAL_FORWARD_TO_RAMP_INCHES = 33.0;
    public static double INITIAL_BACKUP_FROM_START = 7.0;


    // -----------------------------
    // TURN ANGLES (TUNABLE)
    // -----------------------------
    public static double TURN_TO_INTAKE = -53;
    public static double TURN_TO_SHOOT = 46.0;

    // -----------------------------
    // DRIVE POWERS
    // -----------------------------
    public static double DRIVE_POWER_SLOW_INTAKE   = 0.40;

    public static double FINAL_EXIT_1 = -5.0;
    public static double FINAL_EXIT_2 = -15.0;
    public static double FINAL_TURN = 45.0;

    public static double GATEKEEPER_POWER = -1;
    public static long WAIT_BEFORE_4th_BALL = 500; // was 1000; smart routine handles readiness

    @Override
    public void runOpMode() {

        Robot.initializeRobot(hardwareMap);
        Robot.recomputeConstants();
        Robot.activeOpMode = this;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Auto Blue - 6 Ball READY (Smart Shoot)");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        // ============================================================
        // STEP 1 — SPIN SHOOTER FOR PRELOADS
        // ============================================================
        Robot.shooter.setVelocity(SHOOTER_FIRE_VELOCITY);

        // Optional kickstart (enable if you want faster spin-up):
        // Robot.shooter.setPower(1.0);
        // Robot.safeWait(120);
        // Robot.shooter.setVelocity(SHOOTER_FIRE_VELOCITY);

        // Let the filter settle a moment
        Robot.safeWait(100);

        Robot.driveStraightInches(this, -INITIAL_BACKUP_FROM_START, 1.0);

        // ============================================================
        // STEP 2 — SHOOT FIRST 3 BALLS (SMART)
        // ============================================================
        Robot.shoot3PreloadsSmart(SHOOTER_FIRE_VELOCITY);

        // ============================================================
        // STEP 3 — DRIVE TO FIELD BALLS
        // ============================================================
        Robot.driveStraightInches(this, -BACK_UP_FROM_START_INCHES, 1.0);
        Robot.safeWait(200);

        Robot.turnDegreesIMU(this, TURN_TO_INTAKE);
        Robot.safeWait(50);

        // ============================================================
        // STEP 4 — SHOOTER REVERSE WHILE INTAKING
        // ============================================================
        Robot.shooter.setVelocity(SHOOTER_INTAKE_VELOCITY);
        Robot.intakeMotor.setPower(Robot.INTAKE_POWER);
        Robot.leftGatekeeperServo.setPower(GATEKEEPER_POWER);
        Robot.rightGatekeeperServo.setPower(GATEKEEPER_POWER);
        Robot.trapServo.setPosition(Robot.TRAP_OPEN_POS);

        Robot.driveStraightInches(this, FORWARD_AFTER_TURN_INCHES, 1.0);
        Robot.driveStraightSlowInches(this, SLOW_FORWARD_INTAKE_INCHES, DRIVE_POWER_SLOW_INTAKE);

        // ============================================================
        // STEP 5 — RETURN TO SHOOTING POSITION
        // ============================================================
        Robot.intakeMotor.setPower(-0.5);
        Robot.driveStraightInches(this, -BACK_TO_RAMP_INCHES, 1.0);

        Robot.trapServo.setPosition(Robot.TRAP_CLOSED_POS);
        Robot.leftGatekeeperServo.setPower(0);
        Robot.rightGatekeeperServo.setPower(0);
        Robot.intakeMotor.setPower(0);
        Robot.safeWait(200);

        Robot.turnDegreesIMU(this, TURN_TO_SHOOT);
        Robot.safeWait(200);

        Robot.shooter.setVelocity(SHOOTER_FIRE_VELOCITY);
        Robot.driveStraightInches(this, FINAL_FORWARD_TO_RAMP_INCHES, 1.0);

        Robot.safeWait(WAIT_BEFORE_4th_BALL);

        // ============================================================
        // STEP 6 — SHOOT LAST 3 BALLS (SMART)
        // ============================================================
        //Robot.ALLOW_OUTTAKE = false;
        Robot.shoot3FinalSmart(SHOOTER_FIRE_VELOCITY);
        //Robot.ALLOW_OUTTAKE = true;

        // ============================================================
        // STEP 7 — PARK
        // ============================================================
        Robot.driveStraightInches(this, FINAL_EXIT_1, 1.0);
        Robot.turnDegreesIMU(this, FINAL_TURN);
        Robot.safeWait(50);
        Robot.driveStraightInches(this, FINAL_EXIT_2, 1.0);
    }
}
