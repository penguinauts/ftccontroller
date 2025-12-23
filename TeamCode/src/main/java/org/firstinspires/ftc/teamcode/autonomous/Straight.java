package org.firstinspires.ftc.teamcode.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.helpers.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Straight extends LinearOpMode {


    @Override
    public void runOpMode () throws InterruptedException {

        Robot.initializeRobot(hardwareMap);
        telemetry.addLine("Robot ready to go!");
        telemetry.update();

        waitForStart();
//
//
//
//
//
//
//
        Robot.leftWheel.setPower(1);
        Robot.rightWheel.setPower(1);
        sleep(300);
        Robot.leftWheel.setPower(0);
        Robot.rightWheel.setPower(0);










        telemetry.addData("motorSpeed", Robot.shooter.getVelocity());
        telemetry.update();
        Robot.shooter.setPower(0);

    }


}