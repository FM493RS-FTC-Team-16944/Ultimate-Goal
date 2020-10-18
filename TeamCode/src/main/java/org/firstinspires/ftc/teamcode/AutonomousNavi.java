package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="AutoNavi", group="Autonomous")
//@disabled
public class AutonomousNavi extends LinearOpMode {

    DcMotor FrontLeftDrive, FrontRightDrive, BackLeftDrive, BackRightDrive;
    double   FLPower, FRPower, BLPower, BRPower,xValue, yValue;

    @Override
    public void runOpMode() throws InterruptedException
    {
        FrontLeftDrive = hardwareMap.dcMotor.get("FrontLeftDrive");
        FrontRightDrive = hardwareMap.dcMotor.get("FrontRightDrive");
        BackLeftDrive = hardwareMap.dcMotor.get("BackLeftDrive");
        BackRightDrive = hardwareMap.dcMotor.get("BackRightDrive");

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        while (opModeIsActive())
        {

                                 //Place Code Here
    MecanumDirectionalFunction m = new MecanumDirectionalFunction();             //Instantiate object m of class MecanumDirectionalFunction *Object does not have to be called m*
    m.Calculation(0, 1, 1);                  //Calculate
    FrontLeftDrive.setPower(m.GetFrontLeftPower(1));                              //Set Motor powers
    BackLeftDrive.setPower(m.GetBackLeftPower(1));
    FrontRightDrive.setPower(m.GetFrontRightPower(1));
    BackRightDrive.setPower(m.GetBackRightPower(1));
    idle(1s)
        MecanumDirectionalFunction d = new MecanumDirectionalFunction();             //Instantiate object m of class MecanumDirectionalFunction *Object does not have to be called m*
    d.Calculation(4, 0, 1);                  //Calculate
    FrontLeftDrive.setPower(m.GetFrontLeftPower(1));                              //Set Motor powers
    BackLeftDrive.setPower(m.GetBackLeftPower(0));
    FrontRightDrive.setPower(m.GetFrontRightPower(0));
    BackRightDrive.setPower(m.GetBackRightPower(1));






            idle();

        }
    }
}
