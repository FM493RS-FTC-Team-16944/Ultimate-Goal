package org.firstinspires.ftc.teamcode;

import android.text.method.Touch;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


@TeleOp(name="MecanumTOShooter", group="Driver")
//@Disabled
public class MecanumTOShooter extends LinearOpMode {
    DcMotor FrontLeftDrive, FrontRightDrive, BackLeftDrive, BackRightDrive, LeftShooter, RightShooter, ArmBase, Intake;
    //TouchSensor Touch;
    Servo Gripper;
    CRServo RightCRServo, LeftCRServo;
    double   FLPower, FRPower, BLPower, BRPower, ConstRes, IntakePower;
    double LaunchPower;
    boolean LeftBumper, buttonA, RightBumper;
    boolean AutoOn = false;
    boolean AutoSwitch = false;
    boolean AutoIn = false;
    boolean PreviousBumper = false;
    boolean PreviousbuttonA = false;
    boolean PreviousIntake = false;
    float GripStregnth = 0;
    double ArmPower = 0;

    /** Start of Vuforia Init */

    // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
    // NOTE: If running on a CONTROL HUB, with only one USB WebCam, select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;


    private static final String VUFORIA_KEY =
            "AdwpsnD/////AAABmXLkfWRnc0QfuiNxVS6Yh4tOMfYBWgTqMB9KcHBB+YRPnbQLwETIZbsWLSUNI8SoPBEt3Sbi39RHPSQiGerufWT8VnW6WP0iT+tpJfcTdKRYKNa5k9ZQVC4eRJ4ROyersNVLlixPTu6Se6aTmlOhkWGxlSQawoPwJbrtvhSS9YKB1pr95zuC8uKz4luBSgaYU/bVSCgvE7oH2Gzlipi9TQCCp3OoW+kVogZQHxiGlzpGUhGGNC4gH/n4iB5KhAMcersPPedQgRr8hTGLmYIPFC5M6sTxhYPg4CWMyMKSg5kPKUr0uEh9SM5ZoxLdIEw9iUMSkmDRytYmob5A8v1vOI6J9FFHmKvlGFvSGkTNTRMC";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;
    /**End of vuforia Init*/



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
        Intake = hardwareMap.dcMotor.get("Intake");
        ArmBase = hardwareMap.dcMotor.get("ArmBase");
        Gripper = hardwareMap.servo.get("Gripper");
        RightCRServo = hardwareMap.crservo.get("RightCRServo");
        LeftCRServo = hardwareMap.crservo.get("LeftCRServo");
        //Touch = hardwareMap.touchSensor.get("Touch");


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

            FLPower = (y + x + rx);
            BLPower = (y - x + rx);
            FRPower = (-y - x + rx);
            BRPower = (-y + x + rx);

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
                ArmPower = 0.1;
            else ArmPower = 0;

            if (gamepad1.dpad_down)
                ArmPower = -0.4;
            if (gamepad1.dpad_up)
                ArmPower = 0.4;

            /*
            if(Touch.isPressed())
                ConstRes = 0.1;
            else
                ConstRes = 0;

             */

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

            //Start of Intake Code

            RightBumper = gamepad1.right_bumper;

            if (RightBumper == true&&RightBumper!=PreviousIntake)
                AutoIn =  !AutoIn;

            if (AutoIn==true) {
                IntakePower = -1;
                LaunchPower = -0.4;
                RightCRServo.setPower(-1);
                LeftCRServo.setPower(1);
            } else {
                IntakePower = 0;
                RightCRServo.setPower(0);
                LeftCRServo.setPower(0);
            }

            if(gamepad1.right_trigger != 0)
                IntakePower =  -gamepad1.right_trigger;

            PreviousIntake = RightBumper;

            //End of Intake Code

            //Start of PreShoot Sequence
            if (gamepad1.b==true){
                LaunchPower=-0.8;
                IntakePower=0.4;
            }
            //End of PreShoot Sequence

            //Motor Power Assignment
            FrontLeftDrive.setPower(FLPower);
            BackLeftDrive.setPower(BLPower);
            FrontRightDrive.setPower(FRPower);
            BackRightDrive.setPower(BRPower);
            RightShooter.setPower(-LaunchPower);
            LeftShooter.setPower(LaunchPower);
            Intake.setPower(IntakePower);
            ArmBase.setPower(ArmPower);
            //Gripper.setPosition(Range.clip(GripStregnth, 0, 1));
            //End of code
            idle();
        }
    }
}