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
    public static double SHOOTER_SPEED_FORWARD = 1100;
    public static double SHOOTER_SPEED_REVERSE = -200;

    // ----------------------------------------------------
    // DPAD Shooter Presets
    // ----------------------------------------------------
    public static double DPAD_UP = 1225;
    public static double DPAD_DOWN = 1;
    public static double DPAD_RIGHT = 1000;
    public static double DPAD_LEFT = 1275;

    // ----------------------------------------------------
    // SHOOTER CONSISTENCY FEATURES
    // ----------------------------------------------------
    public static boolean ENABLE_VELOCITY_GATING = true;     // Prevent shooting if velocity too low
    public static boolean ENABLE_AUTO_RECOVERY = true;       // Auto-wait for recovery between shots
    public static int MIN_SHOT_INTERVAL_MS = 350;            // Minimum time between shots
    
    // DPAD edge detection
    boolean lastDpadUp = false;
    boolean lastDpadDown = false;
    boolean lastDpadLeft = false;
    boolean lastDpadRight = false;

    // Shooter override state
    boolean shooterManualOverride = false;
    double shooterTargetVelocity = SHOOTER_SPEED_FORWARD;
    
    // Shooter readiness tracking
    private final ElapsedTime lastShotTimer = new ElapsedTime();
    private boolean shooterReady = false;
    private boolean lastShotWasReady = true;

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
        Robot.shooter.setVelocity(shooterTargetVelocity);
        lastShotTimer.reset();

        while (opModeIsActive()) {

            // ----------------------------------------------------
            // CHECK SHOOTER VELOCITY STATUS
            // ----------------------------------------------------
            double currentVelocity = Robot.shooter.getVelocity();
            double velocityError = Math.abs(shooterTargetVelocity - currentVelocity);
            
            // Determine if shooter is ready to fire
            if (shooterTargetVelocity > 0) {  // Forward shooting mode
                shooterReady = velocityError <= Robot.SHOOTER_VELOCITY_TOLERANCE &&
                               lastShotTimer.milliseconds() >= MIN_SHOT_INTERVAL_MS;
            } else {  // Reverse mode (intaking) - always ready
                shooterReady = true;
            }

            // ----------------------------------------------------
            // RIGHT BUMPER RELEASE = Gatekeeper pulse (with velocity gating)
            // ----------------------------------------------------
            boolean rb = gamepad1.right_bumper;

            if (!rightBumperActive && rightBumperPressed && !rb) {
                // Check if shooter is ready before allowing shot
                if (!ENABLE_VELOCITY_GATING || shooterReady) {
                    Robot.leftGatekeeperServo.setPower(1);
                    Robot.rightGatekeeperServo.setPower(1);
                    rightBumperTimer.reset();
                    rightBumperActive = true;
                    lastShotTimer.reset();  // Track shot timing
                    lastShotWasReady = shooterReady;
                } else {
                    // Provide haptic feedback that shooter isn't ready
                    gamepad1.rumble(200);
                }
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
            // TELEMETRY - Enhanced with shooter status
            // ----------------------------------------------------
            telemetry.addLine("========== SHOOTER STATUS ==========");
            telemetry.addData("Target Velocity", "%.0f", shooterTargetVelocity);
            telemetry.addData("Current Velocity", "%.0f", currentVelocity);
            telemetry.addData("Velocity Error", "%.0f", velocityError);
            telemetry.addData("Shooter Ready?", shooterReady ? "✓ READY" : "✗ NOT READY");
            telemetry.addData("Time Since Last Shot", "%.0f ms", lastShotTimer.milliseconds());
            
            // Visual ready indicator
            if (shooterReady) {
                telemetry.addLine("🟢 READY TO SHOOT");
            } else if (velocityError > Robot.SHOOTER_VELOCITY_TOLERANCE) {
                telemetry.addLine("🔴 VELOCITY LOW - WAIT");
            } else {
                telemetry.addLine("🟡 RECOVERY - WAIT");
            }
            
            telemetry.addLine();
            telemetry.addData("Shooter Current (mA)", "%.0f", Robot.shooter.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Manual Override", shooterManualOverride);
            telemetry.addData("Velocity Gating", ENABLE_VELOCITY_GATING ? "ON" : "OFF");
            telemetry.addLine();
            telemetry.addData("Intake Motor Speed", "%.0f", Robot.intakeMotor.getVelocity());
            telemetry.addData("Left Wheel Power", "%.2f", leftPower);
            telemetry.addData("Right Wheel Power", "%.2f", rightPower);
            telemetry.update();

        }
    }
}