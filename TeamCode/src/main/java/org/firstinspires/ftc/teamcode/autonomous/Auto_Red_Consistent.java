package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.helpers.Robot;

@Config
@Autonomous(name = "Auto Red - Consistent", group = "Red Side")
public class Auto_Red_Consistent extends LinearOpMode {

    // === TUNABLE PARAMETERS ===
    public static double BALL1_VELOCITY = 1050;
    public static double BALL2_VELOCITY = 1050;
    public static double BALL3_VELOCITY = 1050;
    public static double BALL4_VELOCITY = 1050;
    public static double BALL5_VELOCITY = 1050;
    public static double BALL6_VELOCITY = 1050;
    public static double TURN_TO_INTAKE = 47.0;
    public static double TURN_TO_SHOOT = -48.0;

    @Override
    public void runOpMode() {
        Robot.initializeRobot(hardwareMap);
        Robot.recomputeConstants();
        Robot.activeOpMode = this;
        Robot.imu.resetYaw();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Auto Red - Consistent READY");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        // Start shooter immediately - it will be ready by ball 1
        Robot.shooter.setVelocity(BALL1_VELOCITY);

        // 1. Shoot preloaded balls
        shootBall1();
        shootBall2();
        shootBall3();

        // 2. Drive to field balls
        Robot.driveStraightInches(this, -41, 1.0);
        Robot.turnDegreesIMU(this, TURN_TO_INTAKE);

        // 3. Intake field balls
        startIntake();
        Robot.driveStraightInches(this, 9, 1.0);
        Robot.driveStraightSlowInches(this, 35, 0.4);

        // 4. Return to shooting position
        Robot.intakeMotor.setPower(-0.5);
        Robot.driveStraightInches(this, -28.5, 1.0);
        stopIntake();

        Robot.turnDegreesIMU(this, TURN_TO_SHOOT);
        Robot.shooter.setVelocity(BALL4_VELOCITY);
        Robot.driveStraightInches(this, 35, 1.0);
        Robot.safeWait(400);

        // 5. Shoot collected balls
        shootBall4();
        shootBall5();
        shootBall6();

        // 6. Park
        Robot.driveStraightInches(this, -5, 1.0);
        Robot.turnDegreesIMU(this, -60);
        Robot.driveStraightInches(this, -22, 1.0);
        Robot.shooter.setVelocity(0);
    }

    // ===========================================
    // BALL 1 - Wait for velocity, then feed with gatekeepers
    // ===========================================
    private void shootBall1() {
        while (opModeIsActive() && Robot.shooter.getVelocity() < BALL1_VELOCITY - 50) {
            idle();
        }
        pulseGatekeepers(400);
        Robot.shooter.setVelocity(BALL2_VELOCITY);
    }

    // ===========================================
    // BALL 2
    // ===========================================
    private void shootBall2() {
        Robot.intakeMotor.setPower(-1);
        Robot.leftGatekeeperServo.setPower(1);
        Robot.rightGatekeeperServo.setPower(1);
        Robot.safeWait(700);
        Robot.intakeMotor.setPower(0);
        Robot.leftGatekeeperServo.setPower(0);
        Robot.rightGatekeeperServo.setPower(0);
        Robot.safeWait(150);
        Robot.shooter.setVelocity(BALL3_VELOCITY);
    }

    // ===========================================
    // BALL 3
    // ===========================================
    private void shootBall3() {
        Robot.OpenAndCloseTheTrapServo();
        Robot.TurnOnOutakeForXMilliSecondsAndTurnOff(50);
        Robot.TurnOnIntakeForXMilliSecondsAndTurnOff(250);
        Robot.safeWait(100);
        pulseGatekeepers(400);
        Robot.safeWait(150);
    }

    // ===========================================
    // BALL 4
    // ===========================================
    private void shootBall4() {
        Robot.TurnOnOutakeForXMilliSecondsAndTurnOff(100);
        Robot.intakeMotor.setPower(-1);
        Robot.leftGatekeeperServo.setPower(1);
        Robot.rightGatekeeperServo.setPower(1);
        Robot.safeWait(600);
        Robot.intakeMotor.setPower(0);
        Robot.leftGatekeeperServo.setPower(0);
        Robot.rightGatekeeperServo.setPower(0);
        Robot.safeWait(250);
        Robot.shooter.setVelocity(BALL5_VELOCITY);
    }

    // ===========================================
    // BALL 5
    // ===========================================
    private void shootBall5() {
        Robot.OpenAndCloseTheTrapServo();
        Robot.TurnOnOutakeForXMilliSecondsAndTurnOff(50);
        Robot.TurnOnIntakeForXMilliSecondsAndTurnOff(250);
        Robot.safeWait(100);
        pulseGatekeepers(400);
        Robot.safeWait(150);
        Robot.shooter.setVelocity(BALL6_VELOCITY);
    }

    // ===========================================
    // BALL 6
    // ===========================================
    private void shootBall6() {
        Robot.OpenAndCloseTheTrapServo();
        Robot.TurnOnOutakeForXMilliSecondsAndTurnOff(50);
        Robot.TurnOnIntakeForXMilliSecondsAndTurnOff(250);
        Robot.safeWait(100);
        pulseGatekeepers(400);
    }

    // ===========================================
    // HELPERS
    // ===========================================

    private void startIntake() {
        Robot.shooter.setVelocity(-200);
        Robot.intakeMotor.setPower(Robot.INTAKE_POWER);
        Robot.leftGatekeeperServo.setPower(-1);
        Robot.rightGatekeeperServo.setPower(-1);
        Robot.trapServo.setPosition(Robot.TRAP_OPEN_POS);
    }

    private void stopIntake() {
        Robot.trapServo.setPosition(Robot.TRAP_CLOSED_POS);
        Robot.leftGatekeeperServo.setPower(0);
        Robot.rightGatekeeperServo.setPower(0);
        Robot.intakeMotor.setPower(0);
    }

    private void pulseGatekeepers(int ms) {
        Robot.leftGatekeeperServo.setPower(1);
        Robot.rightGatekeeperServo.setPower(1);
        Robot.safeWait(ms);
        Robot.leftGatekeeperServo.setPower(0);
        Robot.rightGatekeeperServo.setPower(0);
    }
}
