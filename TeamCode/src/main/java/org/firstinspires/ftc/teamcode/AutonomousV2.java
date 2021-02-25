package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.MecanumDirectionalFunction;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.DashboardUtil;
import org.firstinspires.ftc.teamcode.DashboardUtil;
import org.firstinspires.ftc.teamcode.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.SampleMecanumDrive;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.DriveConstants.kV;


import java.util.List;


@Autonomous(name="AutonomousV2", group="Autonomous")

public class AutonomousV2 extends LinearOpMode {

    //Tensorflow start init
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";


    private static final String VUFORIA_KEY =
            "AdwpsnD/////AAABmXLkfWRnc0QfuiNxVS6Yh4tOMfYBWgTqMB9KcHBB+YRPnbQLwETIZbsWLSUNI8SoPBEt3Sbi39RHPSQiGerufWT8VnW6WP0iT+tpJfcTdKRYKNa5k9ZQVC4eRJ4ROyersNVLlixPTu6Se6aTmlOhkWGxlSQawoPwJbrtvhSS9YKB1pr95zuC8uKz4luBSgaYU/bVSCgvE7oH2Gzlipi9TQCCp3OoW+kVogZQHxiGlzpGUhGGNC4gH/n4iB5KhAMcersPPedQgRr8hTGLmYIPFC5M6sTxhYPg4CWMyMKSg5kPKUr0uEh9SM5ZoxLdIEw9iUMSkmDRytYmob5A8v1vOI6J9FFHmKvlGFvSGkTNTRMC";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    //End of tf init



    //motion init
    DcMotor FrontLeftDrive, FrontRightDrive, BackLeftDrive, BackRightDrive, LeftShooter, RightShooter, Intake, ArmBase;
    Servo Gripper;
    long Motion;
    double   FLPower, FRPower, BLPower, BRPower,xValue, yValue;
    //end of motion init

    @Override
    public void runOpMode() throws InterruptedException
    {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        LeftShooter = hardwareMap.dcMotor.get("LeftShooter");
        RightShooter = hardwareMap.dcMotor.get("RightShooter");
        Intake = hardwareMap.dcMotor.get("Intake");
        ArmBase = hardwareMap.dcMotor.get("ArmBase");
        Gripper = hardwareMap.servo.get("Gripper");


        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }

        Pose2d startPose = new Pose2d(-56, -57, 0);
        drive.setPoseEstimate(startPose);


        // TODO: tune the coordinates to make sure they are accurate and reliable for trajectories, especially for shooting

        Trajectory MovetoRings = drive.trajectoryBuilder(startPose)                  //Moving to rings path
                .splineTo(new Vector2d(3,-58),Math.toRadians(0),               //Go to appropriate distance forward (in front of rings

                        new MinVelocityConstraint(                                      //Restricts the speed of the robot to increase accuracy
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .splineToConstantHeading(new Vector2d(3,-40),Math.toRadians(0),               //Go to appropriate distance right for vuforia

                        new MinVelocityConstraint(                                                  //Restricts the speed of the robot to increase accuracy
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();


        Trajectory PathZero = drive.trajectoryBuilder(MovetoRings.end())             //ZERO path picks off when the first path ends
                .splineTo(new Vector2d(8,-42), Math.toRadians(90))                //Move to first square v
                .build();

        Trajectory PathOne = drive.trajectoryBuilder(MovetoRings.end())             //ONE path picks off when the first path ends
                .splineTo(new Vector2d(30,-42), Math.toRadians(180))                //Move to first square
                .build();

        Trajectory PathTwo = drive.trajectoryBuilder(MovetoRings.end())             //TWO path picks off when the first path ends
                .splineTo(new Vector2d(55,-48), Math.toRadians(90))                //Move to first square
                .build();

        Trajectory BackfromZero = drive.trajectoryBuilder(PathZero.end())           //Move to the line from ZERO
                .strafeTo(new Vector2d(12,-36))
                .build();

        Trajectory BackfromOne = drive.trajectoryBuilder(PathZero.end())           //Move to the line from ONE
                .strafeTo(new Vector2d(12,-36))
                .build();

        Trajectory BackfromTwo = drive.trajectoryBuilder(PathZero.end())          //Move to the line from TWO
                .strafeTo(new Vector2d(12,-36))
                .build();


        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode, servos have been set");
        telemetry.update();
        Gripper.setPosition(Range.clip(0,0,1));



        waitForStart();


        // wait for start button.

        waitForStart();




       //Start of move in front of rings

        drive.followTrajectory(MovetoRings);

        //End of move in front of rings

        /**Tensor flow start: */
        long j = 0;
        int Path = 0;                                          //Base value, end path should be =! 0 if a path is detected

        // TODO: Tune this value to accuratly describe
        while ( j < 3000000) {                //Makes sure that a proper path is returned, if no new value exists, the robot will exit after a set ammount of elapsed time (Determined by # of iterations) T

            if (tfod != null) {
                // getUpdatedjRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());

                        if (recognition.getLabel() == "Quad") {               //Takes the string and determines what type of object it is
                            Path = 2;
                            break;                                              //Exits the loop early if one ring is detected
                        } else if (recognition.getLabel() == "Single") {
                            Path = 1;
                            break;                                             //Exits the loop once the stack of rings is detected
                        }
                    }

                    telemetry.addData("Count: 3,000,000>", j);
                    telemetry.update();


                }
            }

            j++;                                                                //Increases Loop Count
        }



        if (tfod != null) {
            tfod.shutdown();
        }

        telemetry.addData("Path", Path);
        telemetry.update();

        /**Tensor flow end */

        //High goal shoot start

        LeftShooter.setPower(1);
        RightShooter.setPower(-1);
        sleep(500);

        Intake.setPower(-1);

        sleep(3000);

        LeftShooter.setPower(0);
        RightShooter.setPower(0);
        Intake.setPower(0);

//

        //High goal shoot end


        if (Path==0){


            drive.followTrajectory(PathZero);

            // TODO: Make this a seperate method
            sleep(400);
            ArmBase.setPower(-0.4);                                                       //Drops off the wobble goal
            sleep(1500);
            ArmBase.setPower(0);
            Gripper.setPosition(Range.clip(0.5, 0, 1));
            sleep(2000);
            ArmBase.setPower(0.4);
            sleep(1000);

            drive.followTrajectory(BackfromZero);

//

        } else if (Path == 1) {


            drive.followTrajectory(PathOne);

            // TODO: Make this a seperate method
            sleep(400);
            ArmBase.setPower(-0.4);                                                       //Drops off the wobble goal
            sleep(1500);
            ArmBase.setPower(0);
            Gripper.setPosition(Range.clip(0.5, 0, 1));
            sleep(2000);
            ArmBase.setPower(0.4);
            sleep(1000);

            drive.followTrajectory(BackfromOne);


        } else if (Path == 2) {


            drive.followTrajectory(PathTwo);

            // TODO: Make this a seperate method
            sleep(400);
            ArmBase.setPower(-0.4);                                                       //Drops off the wobble goal
            sleep(1500);
            ArmBase.setPower(0);
            Gripper.setPosition(Range.clip(0.5, 0, 1));
            sleep(2000);
            ArmBase.setPower(0.4);
            sleep(1000);

            drive.followTrajectory(BackfromTwo);

        }









    }


        /**
         * Initialize the Vuforia localization engine.
         */
        private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

        /**
         * Initialize the TensorFlow Object Detection engine.
         */
        private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    }

