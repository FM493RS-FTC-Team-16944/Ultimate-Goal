package org.firstinspires.ftc.teamcode;

import android.text.method.Touch;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.lang.annotation.Target;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


@TeleOp(name="MCTOSHINAP", group="Driver")
//@Disabled
public class MCTOSHINAP extends LinearOpMode {
    //Start of std init
    DcMotor FrontLeftDrive, FrontRightDrive, BackLeftDrive, BackRightDrive, LeftShooter, RightShooter, ArmBase, Intake;
    //TouchSensor Touch;
    Servo Gripper;
    double   FLPower, FRPower, BLPower, BRPower, ConstRes, IntakePower;
    float LaunchPower;
    boolean LeftBumper, buttonX, RightBumper;
    boolean AutoOn = false;
    boolean AutoSwitch = false;
    boolean AutoIn = false;
    boolean PreviousBumper = false;
    boolean PreviousbuttonX = false;
    boolean PreviousIntake = false;
    float GripStregnth = 0;
    double ArmPower = 0;
    //End of std Init

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

    //Start of IMU init
    BNO055IMU imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;
    //End of IMU init



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
        //Touch = hardwareMap.touchSensor.get("Touch");


        telemetry.addData("Mode", "waiting");
        telemetry.update();

        //IMU Setup
        BNO055IMU.Parameters Parameters = new BNO055IMU.Parameters();

        Parameters.mode                = BNO055IMU.SensorMode.IMU;
        Parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        Parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        Parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(Parameters);

        //End of IMU setup

        /** Vuforia Setup */
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.

         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * transformation matrices are commonly instances of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        //Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 7.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 2.75f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 3.25f * mmPerInch;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:


        /**End of Vuforia start */

        // wait for start button.

        waitForStart();

        targetsUltimateGoal.activate();        //Activate vuforia

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
            buttonX = gamepad1.a;

            if (gamepad1.x)                 //press X once wobble goal has been grabbed to counteract additional weight
                ArmPower = -0.1;
            else ArmPower = 0;

            if (gamepad1.dpad_down)
                ArmPower = 0.3;
            if (gamepad1.dpad_up)
                ArmPower = -0.4;

            /*
            if(Touch.isPressed())
                ConstRes = 0.1;
            else
                ConstRes = 0;

             */

            ArmPower = ArmPower + ConstRes;

            buttonX = gamepad1.x;

            if (buttonX == true&&buttonX!=PreviousbuttonX)
                AutoSwitch =  !AutoSwitch;

            if (AutoSwitch==true) {
                Gripper.setPosition(Range.clip(0, 0, 1));
            } else {
                Gripper.setPosition(Range.clip(0.5, 0, 1));
            }

            PreviousbuttonX = buttonX;

            //End of Gripper Code

            //Start of Intake Code :)

            RightBumper = gamepad1.right_bumper;

            if (RightBumper == true&&RightBumper!=PreviousIntake)
                AutoIn =  !AutoIn;

            if (AutoIn==true) {
                IntakePower = -1;
            } else {
                IntakePower = 0;
            }

            if(gamepad1.right_trigger != 0)
                IntakePower =  -gamepad1.right_trigger;

            PreviousIntake = RightBumper;

            //End of Intake Code

            /**Vuforia Run */
            double Xpos, Ypos, Zpos, Yaw;
            Xpos = 0;
            Ypos = 0;
            Zpos = 0;
            Yaw = 0;

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }


            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                Xpos = translation.get(0);
                Ypos = translation.get(1);
                Zpos = translation.get(2);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                Yaw = rotation.thirdAngle;
            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();


            /**End of Vuforia Run */

            //AutoAlign Code Start
            if(gamepad1.a==true){
                long speed = 10;
                long robotX = 10;
                long robotY = 10;
                long targetX = 10;
                long targetY = 10;
                long newX = targetX - robotX;
                long newY = targetY - robotY;
                if(newX > 0){
                    long estPX = Math.abs(newX)/speed;
                    MecanumDirectionalFunction posX = new MecanumDirectionalFunction();
                    posX.Calculation(0.8,0 , 0);
                    FrontLeftDrive.setPower(posX.GetFrontLeftPower());
                    BackLeftDrive.setPower(posX.GetBackLeftPower());
                    FrontRightDrive.setPower(posX.GetFrontRightPower());
                    BackRightDrive.setPower(posX.GetBackRightPower());
                    sleep(estPX);
                }else{
                    long estNX = Math.abs(newX)/speed;
                    MecanumDirectionalFunction negX = new MecanumDirectionalFunction();
                    negX.Calculation(-0.8, 0, 0);
                    FrontLeftDrive.setPower(negX.GetFrontLeftPower());
                    BackLeftDrive.setPower(negX.GetBackLeftPower());
                    FrontRightDrive.setPower(negX.GetFrontRightPower());
                    BackRightDrive.setPower(negX.GetBackRightPower());
                    sleep(estNX);
                }
                if(newY > 0){
                    long estPY = Math.abs(newY)/speed;
                    MecanumDirectionalFunction posY = new MecanumDirectionalFunction();
                    posY.Calculation(0, 0.8, 0);
                    FrontLeftDrive.setPower(posY.GetFrontLeftPower());
                    BackLeftDrive.setPower(posY.GetBackLeftPower());
                    FrontRightDrive.setPower(posY.GetFrontRightPower());
                    BackRightDrive.setPower(posY.GetBackRightPower());
                    sleep(estPY);
                }else{
                    long estNY = Math.abs(newY)/speed;
                    MecanumDirectionalFunction negY = new MecanumDirectionalFunction();
                    negY.Calculation(0, -0.8, 0);
                    FrontLeftDrive.setPower(negY.GetFrontLeftPower());
                    BackLeftDrive.setPower(negY.GetBackLeftPower());
                    FrontRightDrive.setPower(negY.GetFrontRightPower());
                    BackRightDrive.setPower(negY.GetBackRightPower());
                    sleep(estNY);
                }
            }

            
            
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
        targetsUltimateGoal.deactivate();

    }


    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
}


    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
}
