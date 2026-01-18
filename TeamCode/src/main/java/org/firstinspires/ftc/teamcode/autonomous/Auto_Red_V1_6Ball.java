package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.helpers.Robot;

@Config
@Autonomous(name="Red Ball Auto", group="Red Side")
public class Auto_Red_V1_6Ball extends LinearOpMode {

    // -----------------------------
    // SHOOTER TUNING
    // -----------------------------
    public static double SHOOTER_FIRE_VELOCITY   = 1100;
    public static double SHOOTER_INTAKE_VELOCITY = -200;

    // -----------------------------
    // DISTANCES (DASHBOARD TUNABLE)
    // -----------------------------
    public static double BACK_UP_FROM_START_INCHES    = 33.0;
    public static double FORWARD_AFTER_TURN_INCHES    = 12;
    public static double SLOW_FORWARD_INTAKE_INCHES   = 23.0;
    public static double BACK_TO_RAMP_INCHES          = 33.0;
    public static double FINAL_FORWARD_TO_RAMP_INCHES = 29.0;
    public static double INITIAL_BACKUP_FROM_START = 7.0;
    public static double INITIAL_TURN_BEFORE_INTAKE = 10.0;


    // -----------------------------
    // TURN ANGLES (MIRRORED)
    //   Blue had: TURN_TO_INTAKE = -51, TURN_TO_SHOOT = +46
    //   Red mirror: +51, -46
    // -----------------------------
    public static double TURN_TO_INTAKE = 60.0;
    public static double TURN_TO_SHOOT  = -46.0;

    // -----------------------------
    // DRIVE POWERS
    // -----------------------------
    public static double DRIVE_POWER_SLOW_INTAKE = 0.40;

    public static double FINAL_EXIT_1 = -0.0;
    public static double FINAL_EXIT_2 = -25.0;

    // Blue final turn was +45, mirrored is -45
    public static double FINAL_TURN = -55.0;

    public static double GATEKEEPER_POWER = -1;
    public static long WAIT_BEFORE_4th_BALL = 400; // was 1000; smart routine handles readiness

    @Override
    public void runOpMode() {

        Robot.initializeRobot(hardwareMap);
        Robot.recomputeConstants();
        Robot.activeOpMode = this;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Auto Red - 6 Ball READY (Smart Shoot)");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        // ============================================================
        // STEP 1 — SPIN SHOOTER FOR PRELOADS
        // ============================================================
        Robot.shooter.setVelocity(SHOOTER_FIRE_VELOCITY);

        // Do movement while shooter spools
        Robot.driveStraightInches(this, -INITIAL_BACKUP_FROM_START, 1.0);

        // Now wait until truly ready, then shoot
        Robot.waitShooterReady(SHOOTER_FIRE_VELOCITY);
        // ============================================================
        // STEP 2 — SHOOT FIRST 3 BALLS (SMART)
        // ============================================================
        Robot.shoot3PreloadsSmart(SHOOTER_FIRE_VELOCITY);

        // ============================================================
        // STEP 3 — DRIVE TO FIELD BALLS
        // ============================================================
        Robot.turnDegreesIMU(this, -INITIAL_TURN_BEFORE_INTAKE);

        Robot.driveStraightInches(this, -BACK_UP_FROM_START_INCHES, 1.0);
        Robot.safeWait(50);

        Robot.turnDegreesIMU(this, TURN_TO_INTAKE);
        //Robot.safeWait(0);

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
        Robot.shooter.setVelocity(SHOOTER_FIRE_VELOCITY);
        Robot.intakeMotor.setPower(-0.5);
        Robot.driveStraightInches(this, -BACK_TO_RAMP_INCHES, 1.0);

        Robot.trapServo.setPosition(Robot.TRAP_CLOSED_POS);
        Robot.leftGatekeeperServo.setPower(0);
        Robot.rightGatekeeperServo.setPower(0);
        Robot.intakeMotor.setPower(0);
        Robot.safeWait(50);

        Robot.turnDegreesIMU(this, TURN_TO_SHOOT);
        //Robot.safeWait(50);

        Robot.driveStraightInches(this, FINAL_FORWARD_TO_RAMP_INCHES, 1.0);

        //Robot.safeWait(WAIT_BEFORE_4th_BALL);

        // ============================================================
        // STEP 6 — SHOOT LAST 3 BALLS (SMART)
        // ============================================================
        Robot.waitShooterReady(SHOOTER_FIRE_VELOCITY);
        Robot.shoot3FinalSmart(SHOOTER_FIRE_VELOCITY);

        // ============================================================
        // STEP 7 — PARK
        // ============================================================
        Robot.driveStraightInches(this, FINAL_EXIT_1, 1.0);
        Robot.turnDegreesIMU(this, FINAL_TURN);
       // Robot.safeWait(50);
        Robot.driveStraightInches(this, FINAL_EXIT_2, 1.0);
    }
}
