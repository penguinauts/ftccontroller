package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.helpers.Robot;

@Config
@TeleOp(name="PenguinautsTeleOp", group="Penguinauts")
public class PenguinautsTeleOp extends LinearOpMode {

    // ----------------------------------------------------
    // SHOOTER VELOCITIES
    // ----------------------------------------------------
    public static double SHOOTER_SPEED_FORWARD = 1200;
    public static double SHOOTER_SPEED_REVERSE = -200;

    // ----------------------------------------------------
    // DPAD Shooter Presets
    // ----------------------------------------------------
    public static double DPAD_UP = 1225;
    public static double DPAD_DOWN = 1;
    public static double DPAD_RIGHT = 1000;
    public static double DPAD_LEFT = 1275;

    // DPAD edge detection
    boolean lastDpadUp = false;
    boolean lastDpadDown = false;
    boolean lastDpadLeft = false;
    boolean lastDpadRight = false;

    // Shooter override state
    boolean shooterManualOverride = false;
    double shooterTargetVelocity = SHOOTER_SPEED_FORWARD;

    // ----------------------------------------------------
    // GATEKEEPER / TRAP TIMING
    // ----------------------------------------------------
    public static int GATEKEEPER_PULSE_MS = 1000;
    public static int TRAP_OPEN_TIME_MS  = 300;
    public static int TRAP_CLOSE_TIME_MS = 50;

    public static double GATEKEEPER_NEG = -1;
    public static double GATEKEEPER_POS = -1;


    boolean rightBumperPressed = false;
    boolean leftBumperPressed  = false;

    double rightTrigger = 0;
    double leftTrigger  = 0;

    private final ElapsedTime rightBumperTimer = new ElapsedTime();
    private boolean rightBumperActive = false;

    @Override
    public void runOpMode () {

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        Robot.initializeRobot(hardwareMap);
        Robot.activeOpMode = this;

        telemetry.addLine("Robot ready to go!");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        // Shooter starts warmed up
        shooterTargetVelocity = SHOOTER_SPEED_FORWARD;

        while (opModeIsActive()) {

            // ----------------------------------------------------
            // RIGHT BUMPER RELEASE = Gatekeeper pulse
            // ----------------------------------------------------
            boolean rb = gamepad1.right_bumper;

            if (!rightBumperActive && rightBumperPressed && !rb) {
                Robot.leftGatekeeperServo.setPower(1);
                Robot.rightGatekeeperServo.setPower(1);
                rightBumperTimer.reset();
                rightBumperActive = true;
            }

            if (rightBumperActive) {
                if (rightBumperTimer.milliseconds() >= GATEKEEPER_PULSE_MS) {
                    Robot.leftGatekeeperServo.setPower(0);
                    Robot.rightGatekeeperServo.setPower(0);
                    rightBumperActive = false;
                }
            }

            rightBumperPressed = rb;

            // ----------------------------------------------------
            // LEFT BUMPER = Intake + Shooter Reverse + Trap Open
            // ----------------------------------------------------
            boolean lb = gamepad1.left_bumper;
            double intakePower = 0;

            if (lb) {
                intakePower = Robot.INTAKE_POWER;
                shooterTargetVelocity = SHOOTER_SPEED_REVERSE;
                shooterManualOverride = false; // override off while reversing
                Robot.leftGatekeeperServo.setPower(GATEKEEPER_NEG);
                Robot.rightGatekeeperServo.setPower(GATEKEEPER_POS);
                Robot.trapServo.setPosition(Robot.TRAP_OPEN_POS);
            } else {
                // Only restore default shooter speed IF driver isn't using DPAD
                if (!shooterManualOverride) {
                    shooterTargetVelocity = SHOOTER_SPEED_FORWARD;
                }

                Robot.trapServo.setPosition(Robot.TRAP_CLOSED_POS);
            }

            leftBumperPressed = lb;

            // ----------------------------------------------------
            // TRIGGERS = intake logic
            // ----------------------------------------------------
            rightTrigger = gamepad1.right_trigger;
            leftTrigger  = gamepad1.left_trigger;

//            if (rightTrigger > 0.1 && leftTrigger > 0.1) {
//                Robot.intakeMotor.setVelocity(0);
//            }
            if (rightTrigger > 0.1) {
                intakePower = Robot.INTAKE_POWER;
            } else if (leftTrigger > 0.1) {
                intakePower = -Robot.INTAKE_POWER;
            }
            Robot.intakeMotor.setPower(intakePower);

            // ----------------------------------------------------
            // BUTTON Y = shooter forward (override DPAD)
            // ----------------------------------------------------
            if (gamepad1.y) {
                shooterTargetVelocity = SHOOTER_SPEED_FORWARD;
                shooterManualOverride = false;
            }

            // ----------------------------------------------------
            // DPAD SHOOTER PRESETS (ONE TAP)
            // ----------------------------------------------------
            if (gamepad1.dpad_up && !lastDpadUp) {
                shooterTargetVelocity = DPAD_UP;
                shooterManualOverride = true;
            }
            if (gamepad1.dpad_down && !lastDpadDown) {
                shooterTargetVelocity = DPAD_DOWN;
                shooterManualOverride = true;
            }
            if (gamepad1.dpad_left && !lastDpadLeft) {
                shooterTargetVelocity = DPAD_LEFT;
                shooterManualOverride = true;
            }
            if (gamepad1.dpad_right && !lastDpadRight) {
                shooterTargetVelocity = DPAD_RIGHT;
                shooterManualOverride = true;
            }

            // Update last DPAD states
            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;
            lastDpadLeft = gamepad1.dpad_left;
            lastDpadRight = gamepad1.dpad_right;

            // ----------------------------------------------------
            // BUTTON B = fire trapdoor
            // ----------------------------------------------------
            if (gamepad1.b) {
                Robot.trapServo.setPosition(Robot.TRAP_OPEN_POS);
                Robot.safeWait(TRAP_OPEN_TIME_MS);
                Robot.trapServo.setPosition(Robot.TRAP_CLOSED_POS);
                Robot.safeWait(TRAP_CLOSE_TIME_MS);
            }

            // ----------------------------------------------------
            // BUTTON X = reverse gatekeepers
            // ----------------------------------------------------
            if (gamepad1.x) {
                Robot.leftGatekeeperServo.setPower(-1);
                Robot.rightGatekeeperServo.setPower(-1);
            } else if (!rightBumperActive) {
                Robot.leftGatekeeperServo.setPower(0);
                Robot.rightGatekeeperServo.setPower(0);
            }
//            else if (gamepad1.a) {
//                Robot.shooter.setVelocity(1);
//            }

            // ----------------------------------------------------
            // TANK DRIVE
            // ----------------------------------------------------
            double leftPower  = -gamepad1.left_stick_y;
            double rightPower = -gamepad1.right_stick_y;

            Robot.leftWheel.setPower(leftPower);
            Robot.rightWheel.setPower(rightPower);

            // ----------------------------------------------------
            // APPLY SHOOTER VELOCITY (once per loop)
            // ----------------------------------------------------
            Robot.shooter.setVelocity(shooterTargetVelocity);

            // ----------------------------------------------------
            // TELEMETRY
            // ----------------------------------------------------
            telemetry.addData("Shooter Target", shooterTargetVelocity);
            telemetry.addData("Shooter Velocity", Robot.shooter.getVelocity());
            telemetry.addData("Shooter Current (mA)", Robot.shooter.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Manual Override", shooterManualOverride);
            telemetry.addData("Intake Motor Speed", Robot.intakeMotor.getVelocity());
            telemetry.update();

        }
    }
}