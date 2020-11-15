package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name="WobGoal", group="Autonomous")

public class WobbleGoal extends LinearOpMode {

    DcMotor FrontLeftDrive, FrontRightDrive, BackLeftDrive, BackRightDrive, LeftShooter, RightShooter, ArmBase;
    Servo Gripper;
    double   FLPower, FRPower, BLPower, BRPower,xValue, yValue;

    @Override
    public void runOpMode() throws InterruptedException
    {
        FrontLeftDrive = hardwareMap.dcMotor.get("FrontLeftDrive");
        FrontRightDrive = hardwareMap.dcMotor.get("FrontRightDrive");
        BackLeftDrive = hardwareMap.dcMotor.get("BackLeftDrive");
        BackRightDrive = hardwareMap.dcMotor.get("BackRightDrive");
        LeftShooter = hardwareMap.dcMotor.get("LeftShooter");
        RightShooter = hardwareMap.dcMotor.get("RightShooter");
        ArmBase = hardwareMap.dcMotor.get("ArmBase");
        Gripper = hardwareMap.servo.get("Gripper");


        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();


        //Pick Up Goal
        Gripper.setPosition(Range.clip(0.5, 0, 1));
        sleep(2000);
        ArmBase.setPower(0.3);
        sleep(850);
        ArmBase.setPower(-0.1);
        sleep(750);
        Gripper.setPosition(Range.clip(0,0,1));
        sleep(2000);
        ArmBase.setPower(-0.4);
        Gripper.setPosition(Range.clip(0,0,1));
        sleep(800);
        ArmBase.setPower(-0.2);
        Gripper.setPosition(Range.clip(0,0,1));
        sleep(400);
        ArmBase.setPower(-0.1);
        Gripper.setPosition(Range.clip(0,0,1));
        
        
        m.Calculation(0, -0.5, 0);                              //Calculate
        FrontLeftDrive.setPower(m.GetFrontLeftPower());                              //Set Motor powers
        BackLeftDrive.setPower(m.GetBackLeftPower());
        FrontRightDrive.setPower(m.GetFrontRightPower());
        BackRightDrive.setPower(m.GetBackRightPower());
        sleep(200); 

        






        idle();





    }
}
