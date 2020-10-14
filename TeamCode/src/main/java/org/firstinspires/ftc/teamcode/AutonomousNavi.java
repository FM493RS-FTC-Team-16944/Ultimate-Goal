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








            idle();

        }
    }
}
