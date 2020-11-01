package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="MecanumTOShooter", group="Driver")
//@Disabled
public class MecanumTOShooter extends LinearOpMode {
    DcMotor FrontLeftDrive, FrontRightDrive, BackLeftDrive, BackRightDrive, LeftShooter, RightShooter, ArmBase;
    TouchSensor Touch;
    Servo Gripper;
    double   FLPower, FRPower, BLPower, BRPower, ConstRes;
    float LaunchPower;
    boolean LeftBumper, buttonA;
    boolean AutoOn = false;
    boolean AutoSwitch = false;
    boolean PreviousBumper = false;
    boolean PreviousbuttonA = false;
    float GripStregnth = 0;
    double ArmPower = 0;


    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
       //Hardware Mapping
        FrontLeftDrive = hardwareMap.dcMotor.get("FrontLeftDrive");
        FrontRightDrive = hardwareMap.dcMotor.get("FrontRightDrive");
        BackLeftDrive = hardwareMap.dcMotor.get("BackLeftDrive");
        BackRightDrive = hardwareMap.dcMotor.get("BackRightDrive");
        LeftShooter = hardwareMap.dcMotor.get("LeftShooter");
        RightShooter = hardwareMap.dcMotor.get("RightShooter");
        ArmBase = hardwareMap.dcMotor.get("ArmBase");
        Gripper = hardwareMap.servo.get("Gripper");
        Touch = hardwareMap.touchSensor.get("Touch");


        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        while (opModeIsActive())
        {

            //Start of Drive Code
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            FLPower = (y - x - rx);
            BLPower = (- y - x + rx);
            FRPower = (y + x + rx);
            BRPower = (y - x + rx);

            telemetry.addData("Mode", "running");
            telemetry.addData("stick", "  y=" + y + "  x=" + x);
            telemetry.update();

            if (Math.abs(FLPower) > 1 || Math.abs(BLPower) > 1 ||
                    Math.abs(FRPower) > 1 || Math.abs(BRPower) > 1 ) {
                // Find the largest power

                double max = 0;
                max = Math.max(Math.abs(FLPower), Math.abs(BLPower));
                max = Math.max(Math.abs(FRPower), max);
                max = Math.max(Math.abs(BRPower), max);

                // Divide everything by max (it's positive so we don't need to worry
                // about signs)
                FLPower /= max;
                BLPower /= max;
                FRPower /= max;
                BRPower /= max;

                telemetry.addData("PowerScaling Coeficcient:", 1/max);
            } else {
                telemetry.addData("PowerScaling Coeficcient:", "N/A");
            }
            //End of drive Code

            //Start of Shooter Code


            LeftBumper = gamepad1.left_bumper;

            if (LeftBumper == true&&LeftBumper!=PreviousBumper)
                AutoOn =  !AutoOn;

            if (AutoOn==true) {
                LaunchPower = 1;
            } else {
                LaunchPower = 0;
            }

            if(gamepad1.left_trigger != 0)
                LaunchPower =  gamepad1.left_trigger;

            PreviousBumper = LeftBumper;

            //End of Shooter Code

            //Start of Gripper Code
            buttonA = gamepad1.a;

            if (gamepad1.x)                 //press X once wobble goal has been grabbed to counteract additional weight
                ArmPower = -0.1;
            else ArmPower = 0;

            if (gamepad1.dpad_down)
                ArmPower = 0.3;
            if (gamepad1.dpad_up)
                ArmPower = -0.4;

            if(Touch.isPressed())
                ConstRes = 0.1;
            else
                ConstRes = 0;

            ArmPower = ArmPower + ConstRes;

            buttonA = gamepad1.a;

            if (buttonA == true&&buttonA!=PreviousbuttonA)
                AutoSwitch =  !AutoSwitch;

            if (AutoSwitch==true) {
                Gripper.setPosition(Range.clip(0, 0, 1));
            } else {
                Gripper.setPosition(Range.clip(0.5, 0, 1));
            }

            PreviousbuttonA = buttonA;

            //End of Gripper Code

            //Motor Power Assignment
            FrontLeftDrive.setPower(FLPower);
            BackLeftDrive.setPower(BLPower);
            FrontRightDrive.setPower(FRPower);
            BackRightDrive.setPower(BRPower);
            RightShooter.setPower(-LaunchPower);
            LeftShooter.setPower(LaunchPower);
            ArmBase.setPower(ArmPower);
            //Gripper.setPosition(Range.clip(GripStregnth, 0, 1));
            //End of code
            idle();
        }
    }
}