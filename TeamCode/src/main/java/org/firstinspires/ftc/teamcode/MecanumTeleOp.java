package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="jjMCN", group="Driver")
//@Disabled
public class MecanumTeleOp extends LinearOpMode {
    DcMotor FrontLeftDrive, FrontRightDrive, BackLeftDrive, BackRightDrive;
    double   FLPower, FRPower, BLPower, BRPower,xValue, yValue;

    // called when init button is  pressed.
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
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

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

            FrontLeftDrive.setPower(FLPower);
            BackLeftDrive.setPower(BLPower);
            FrontRightDrive.setPower(FRPower);
            BackRightDrive.setPower(BRPower);

            idle();
        }
    }
}