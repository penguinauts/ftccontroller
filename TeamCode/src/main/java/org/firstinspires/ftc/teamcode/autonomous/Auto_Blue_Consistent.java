package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.helpers.Robot.imu;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.helpers.Robot;

@Config
@Autonomous(name = "Auto Blue - Consistent", group = "Blue Side")
public class Auto_Blue_Consistent extends LinearOpMode {

    // === TUNABLE PARAMETERS ===
    public static double BALL1_VELOCITY = 1100;
    public static double BALL2_VELOCITY = 1100;
    public static double BALL3_VELOCITY = 1100;
    public static double BALL4_VELOCITY = 1115;
    public static double BALL5_VELOCITY = 1095;
    public static double BALL6_VELOCITY = 1095;

    @Override
    public void runOpMode() {
        Robot.initializeRobot(hardwareMap);
        Robot.recomputeConstants();
        Robot.activeOpMode = this;
        imu.resetYaw();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Auto Blue - Consistent READY");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;


       Robot.shooter.setVelocity(BALL1_VELOCITY);
        Robot.driveStraightInches(this, -2.5, 1.0);
       // 1. Shoot preloaded balls
       shootBall1();
       shootBall2();
       shootBall3();

        // 2. Drive to field balls
        Robot.driveStraightInches(this, -4, 1.0);
        Robot.turnDegreesIMU(this, 10);
        Robot.driveStraightInches(this, -29.3, 1.0);
        Robot.turnDegreesIMU(this, -58);

         // 3. Intake field balls
         Robot.shooter.setVelocity(-200);
         Robot.intakeMotor.setPower(-1.0);
         Robot.leftGatekeeperServo.setPower(-1);
         Robot.rightGatekeeperServo.setPower(-1);
         Robot.trapServo.setPosition(Robot.TRAP_OPEN_POS);
         Robot.intakeMotor.setPower(-1);
        Robot.driveStraightInches(this, 6.5, 1.0);
         Robot.driveStraightSlowInches(this, 13, 0.1);

        Robot.trapServo.setPosition(Robot.TRAP_CLOSED_POS);
        Robot.leftGatekeeperServo.setPower(0);
        Robot.rightGatekeeperServo.setPower(0);
        Robot.intakeMotor.setPower(0);

        Robot.shooter.setVelocity(BALL4_VELOCITY);

        Robot.turnDegreesIMU(this, 90);
        Robot.driveStraightInches(this, 20, 1.0);
        Robot.turnDegreesIMU(this, -45);
        Robot.driveStraightInches(this, -5, 1.0);
        shootBall4();
        shootBall5();
        shootBall6();

         Robot.driveStraightInches(this, -3, 1.0);
         Robot.turnDegreesIMU(this, 45);
         Robot.driveStraightInches(this, -15, 1.0);
         Robot.shooter.setVelocity(0);
    }

    // ===========================================
    // BALL 1 - Wait for velocity, then feed with gatekeepers
    // ===========================================
    private void shootBall1() {
        while (opModeIsActive() && (Robot.shooter.getVelocity() < BALL1_VELOCITY - 10)) {
            idle();
        }
        telemetry.addLine("shootBall1 velocity" + Robot.shooter.getVelocity());
        telemetry.update();
        pulseGatekeepers(400, 1);
        Robot.shooter.setVelocity(BALL2_VELOCITY);
    }

    // ===========================================
    // BALL 2
    // ===========================================
    private void shootBall2() {
        while (opModeIsActive() && (Robot.shooter.getVelocity() < BALL2_VELOCITY - 10)) {
            idle();
        }
        telemetry.addLine("shootBall2 velocity" + Robot.shooter.getVelocity());
        telemetry.update();
        Robot.intakeMotor.setPower(-1);
        pulseGatekeepers(1000, 1);
        Robot.intakeMotor.setPower(0);
        Robot.shooter.setVelocity(BALL3_VELOCITY);
    }

    // ===========================================
    // BALL 3
    // ===========================================
    private void shootBall3() {
        while (opModeIsActive() && (Robot.shooter.getVelocity() < BALL3_VELOCITY - 10)) {
            idle();
        }
        telemetry.addLine("shootBall3 velocity" + Robot.shooter.getVelocity());
        telemetry.update();
        Robot.OpenAndCloseTheTrapServo();
        Robot.intakeMotor.setPower(1);
        pulseGatekeepers(225, -1);
        Robot.intakeMotor.setPower(-1);
        pulseGatekeepers(1500, 1);
        Robot.intakeMotor.setPower(0);
        Robot.shooter.setVelocity(BALL4_VELOCITY);
    }

    // ===========================================
    // BALL 4
    // ===========================================
    private void shootBall4() {
        //Robot.shooter.setVelocity(BALL4_VELOCITY);
        while (opModeIsActive() && (Robot.shooter.getVelocity() < BALL4_VELOCITY - 10)) {
            idle();
        }
        telemetry.addLine("shootBall4 velocity" + Robot.shooter.getVelocity());
        telemetry.update();
        Robot.intakeMotor.setPower(1);
        pulseGatekeepers(100, -1);
        Robot.intakeMotor.setPower(-1);
        pulseGatekeepers(1000, 1);
        Robot.intakeMotor.setPower(0);
        Robot.shooter.setVelocity(BALL5_VELOCITY);

    }

    // ===========================================
    // BALL 5
    // ===========================================
    private void shootBall5() {
        while (opModeIsActive() && (Robot.shooter.getVelocity() < BALL5_VELOCITY - 10)) {
            idle();
        }
        telemetry.addLine("shootBall5 velocity" + Robot.shooter.getVelocity());
        telemetry.update();
        Robot.OpenAndCloseTheTrapServo();
        Robot.intakeMotor.setPower(1);
        pulseGatekeepers(100, -1);
        Robot.intakeMotor.setPower(-1);
        pulseGatekeepers(1500, 1);
        Robot.intakeMotor.setPower(0);
        Robot.shooter.setVelocity(BALL6_VELOCITY);
    }

    // ===========================================
    // BALL 6
    // ===========================================
    private void shootBall6() {
        while (opModeIsActive() && (Robot.shooter.getVelocity() < BALL6_VELOCITY - 10)) {
            idle();
        }
        telemetry.addLine("shootBall5 velocity" + Robot.shooter.getVelocity());
        telemetry.update();
        Robot.OpenAndCloseTheTrapServo();
        Robot.intakeMotor.setPower(1);
        pulseGatekeepers(100, -1);
        Robot.intakeMotor.setPower(-1);
        pulseGatekeepers(1500, 1);
        Robot.intakeMotor.setPower(0);
    }

    // ===========================================
    // HELPERS
    // ===========================================
    private void pulseGatekeepers(int ms, int power) {
        Robot.leftGatekeeperServo.setPower(power);
        Robot.rightGatekeeperServo.setPower(power);
        Robot.safeWait(ms);
        Robot.leftGatekeeperServo.setPower(0);
        Robot.rightGatekeeperServo.setPower(0);
    }
}
