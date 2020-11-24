package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="AutoNavi", group="Autonomous")
@Disabled
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

        while (opModeIsActive()) {

            telemetry.addLine("hee");
            telemetry.update();

            MecanumDirectionalFunction m = new MecanumDirectionalFunction();             //Instantiate object m of class MecanumDirectionalFunction *Object does not have to be called m*
            m.Calculation(0, 1, 0);                              //Calculate
            FrontLeftDrive.setPower(m.GetFrontLeftPower());                              //Set Motor powers
            BackLeftDrive.setPower(m.GetBackLeftPower());
            FrontRightDrive.setPower(m.GetFrontRightPower());
            BackRightDrive.setPower(m.GetBackRightPower());

            telemetry.addLine("2");
            telemetry.update();

            //wait(1000);                                                            //Adjust forward Movement
            sleep(500);

            telemetry.addLine("3");
            telemetry.update();

            MecanumDirectionalFunction c = new MecanumDirectionalFunction();             //Instantiate object m of class MecanumDirectionalFunction *Object does not have to be called m*
            c.Calculation(0, 0, 0);                              //Calculate
            FrontLeftDrive.setPower(c.GetFrontLeftPower());                              //Set Motor powers
            BackLeftDrive.setPower(c.GetBackLeftPower());
            FrontRightDrive.setPower(c.GetFrontRightPower());
            BackRightDrive.setPower(c.GetBackRightPower());

            MecanumDirectionalFunction n = new MecanumDirectionalFunction();             //Instantiate object m of class MecanumDirectionalFunction *Object does not have to be called m*
            n.Calculation(-1, 0, 0);                                 //Calculate
            FrontLeftDrive.setPower(n.GetFrontLeftPower());                                 //Set Motor powers
            BackLeftDrive.setPower(n.GetBackLeftPower());
            FrontRightDrive.setPower(n.GetFrontRightPower());
            BackRightDrive.setPower(n.GetBackRightPower());

            telemetry.addLine("4");
            telemetry.update();

            //wait(4000);                                                             //Adjust Sideways Movement
            sleep(750);

            telemetry.addLine("5");
            telemetry.update();

            MecanumDirectionalFunction d = new MecanumDirectionalFunction();             //Instantiate object m of class MecanumDirectionalFunction *Object does not have to be called m*
            d.Calculation(0, 0, 0);                              //Calculate
            FrontLeftDrive.setPower(d.GetFrontLeftPower());                              //Set Motor powers
            BackLeftDrive.setPower(d.GetBackLeftPower());
            FrontRightDrive.setPower(d.GetFrontRightPower());
            BackRightDrive.setPower(d.GetBackRightPower());

            telemetry.addLine("6");
            telemetry.update();

            //Place Code Here




            idle();

        }
    }
}
