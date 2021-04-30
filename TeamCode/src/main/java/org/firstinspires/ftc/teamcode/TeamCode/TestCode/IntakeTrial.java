package org.firstinspires.ftc.teamcode.TeamCode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name="IntakeTrial", group="Autonomous")

public class IntakeTrial extends LinearOpMode {


    //motion init
    DcMotor FrontLeftDrive, FrontRightDrive, BackLeftDrive, BackRightDrive, LeftShooter, RightShooter, Intake,ArmBase;
    Servo Gripper;
    long Motion;
    double FLPower, FRPower, BLPower, BRPower, xValue, yValue;
    //end of motion init

    @Override
    public void runOpMode() throws InterruptedException {
        /*FrontLeftDrive = hardwareMap.dcMotor.get("FrontLeftDrive");
        FrontRightDrive = hardwareMap.dcMotor.get("FrontRightDrive");
        BackLeftDrive = hardwareMap.dcMotor.get("BackLeftDrive");
        BackRightDrive = hardwareMap.dcMotor.get("BackRightDrive");
        LeftShooter = hardwareMap.dcMotor.get("LeftShooter");
        RightShooter = hardwareMap.dcMotor.get("RightShooter"); */
        Intake = hardwareMap.dcMotor.get("Intake");
        //ArmBase = hardwareMap.dcMotor.get("ArmBase");
        //Gripper = hardwareMap.servo.get("Gripper");


        telemetry.addData("Mode", "waiting");
        telemetry.update();

        waitForStart();


        //LeftShooter.setPower(1);
        //RightShooter.setPower(-1);
        Intake.setPower(-1);

        sleep(10000);

        //LeftShooter.setPower(0);
        //RightShooter.setPower(0);
        Intake.setPower(0);

    }
}

