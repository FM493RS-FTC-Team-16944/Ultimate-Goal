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
    m.Calculation(0, 1, 0);                              //Calculate
    FrontLeftDrive.setPower(m.GetFrontLeftPower());                              //Set Motor powers
    BackLeftDrive.setPower(m.GetBackLeftPower());
    FrontRightDrive.setPower(m.GetFrontRightPower());
    BackRightDrive.setPower(m.GetBackRightPower());


    wait(1000);                                                            //Adjust forward Movement


        MecanumDirectionalFunction n = new MecanumDirectionalFunction();             //Instantiate object m of class MecanumDirectionalFunction *Object does not have to be called m*
    n.Calculation(-1, 0, 0);                                 //Calculate
    FrontLeftDrive.setPower(n.GetFrontLeftPower());                                 //Set Motor powers
    BackLeftDrive.setPower(n.GetBackLeftPower());
    FrontRightDrive.setPower(n.GetFrontRightPower());
    BackRightDrive.setPower(n.GetBackRightPower());


    wait(4000);                                                             //Adjust Sideways Movement


    MecanumDirectionalFunction d = new MecanumDirectionalFunction();             //Instantiate object m of class MecanumDirectionalFunction *Object does not have to be called m*
    d.Calculation(0, 0, 0);                              //Calculate
    FrontLeftDrive.setPower(d.GetFrontLeftPower());                              //Set Motor powers
    BackLeftDrive.setPower(d.GetBackLeftPower());
    FrontRightDrive.setPower(d.GetFrontRightPower());
    BackRightDrive.setPower(d.GetBackRightPower());




            idle();

        }
    }
}
