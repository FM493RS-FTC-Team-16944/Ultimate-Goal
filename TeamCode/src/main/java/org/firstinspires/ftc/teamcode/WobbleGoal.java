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
        sleep(800);
        ArmBase.setPower(-0.1);
        sleep(750);
        Gripper.setPosition(Range.clip(0,0,1));
        sleep(2000);
        ArmBase.setPower(-0.4);
        sleep(1000);
        ArmBase.setPower(-0.1);

        //Place Code Here
        /*MecanumDirectionalFunction m = new MecanumDirectionalFunction();             //Instantiate object m of class MecanumDirectionalFunction *Object does not have to be called m*
        m.Calculation(0, 1, 0);                              //Calculate
        FrontLeftDrive.setPower(m.GetFrontLeftPower());                              //Set Motor powers
        BackLeftDrive.setPower(m.GetBackLeftPower());
        FrontRightDrive.setPower(m.GetFrontRightPower());
        BackRightDrive.setPower(m.GetBackRightPower());

        sleep(1000);                                                            //Adjust forward Movement


        MecanumDirectionalFunction n = new MecanumDirectionalFunction();             //Instantiate object m of class MecanumDirectionalFunction *Object does not have to be called m*
        n.Calculation(1, 0, 0);                                 //Calculate
        FrontLeftDrive.setPower(n.GetFrontLeftPower());                                 //Set Motor powers
        BackLeftDrive.setPower(n.GetBackLeftPower());
        FrontRightDrive.setPower(n.GetFrontRightPower());
        BackRightDrive.setPower(n.GetBackRightPower());


        sleep(5000);     //Adjust Sideways Movement

         */






        idle();





    }
}
