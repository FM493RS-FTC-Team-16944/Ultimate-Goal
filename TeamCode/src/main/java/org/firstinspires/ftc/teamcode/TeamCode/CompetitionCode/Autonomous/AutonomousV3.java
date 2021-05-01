package org.firstinspires.ftc.teamcode.TeamCode.CompetitionCode.Autonomous;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Localization.DriveConstants;

import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;

import org.firstinspires.ftc.teamcode.Localization.SampleMecanumDrive;

import java.util.Arrays;
import java.util.List;


@Autonomous(name="AutonomousV3", group="Autonomous")

public class AutonomousV3 extends LinearOpMode {

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
    DcMotor LeftShooter, RightShooter, Intake, ArmBase;
    Servo GripperA, GripperB;
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
        GripperA = hardwareMap.servo.get("GripperA");
        GripperB = hardwareMap.servo.get("GripperB");


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

        Trajectory BeforeShooting = drive.trajectoryBuilder(startPose)                  //Moving to rings path
                .lineToSplineHeading(new Pose2d(-8,-55,Math.toRadians(20)),               //Go to appropriate distance forward (in front of rings

                        new MinVelocityConstraint(                                      //Restricts the speed of the robot to increase accuracy
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();

        Trajectory DuringShooting = drive.trajectoryBuilder(BeforeShooting.end())

                .lineToSplineHeading(new Pose2d(-8, -54, Math.toRadians(30)),               //Go to appropriate distance right for vuforia

                        new MinVelocityConstraint(                                                  //Restricts the speed of the robot to increase accuracy
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .lineToSplineHeading(new Pose2d(-8, -53.5, Math.toRadians(40)),               //Go to appropriate distance right for vuforia

                        new MinVelocityConstraint(                                                  //Restricts the speed of the robot to increase accuracy
                                Arrays.asList(
                                        new AngularVelocityConstraint(Math.toRadians(2500)),              //Restricts angular velocity to ~50%, to make shooting more even
                                        new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();

        Trajectory MovetoRings = drive.trajectoryBuilder(DuringShooting.end())

                .splineToLinearHeading(new Pose2d(3,-40),Math.toRadians(0),               //Go to appropriate distance right for vuforia

                        new MinVelocityConstraint(                                                  //Restricts the speed of the robot to increase accuracy
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();


        Trajectory PathZero = drive.trajectoryBuilder(MovetoRings.end())             //ZERO path picks off when the first path ends
                .splineTo(new Vector2d(8,-42), Math.toRadians(90))                //Move to first square v
                .build();

        Trajectory PathOne = drive.trajectoryBuilder(MovetoRings.end())             //ONE path picks off when the first path ends
                .splineTo(new Vector2d(25,-42), Math.toRadians(179))                //Move to first square
                .build();

        Trajectory PathTwo = drive.trajectoryBuilder(MovetoRings.end())             //TWO path picks off when the first path ends
                .splineTo(new Vector2d(55,-42), Math.toRadians(90))                //Move to first square
                .build();

        Trajectory BackfromZero = drive.trajectoryBuilder(PathZero.end())           //Move to the wobble goal from ZERO
                .splineToSplineHeading(new Pose2d(-5,-14), Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-30, -15, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Trajectory BackfromOne = drive.trajectoryBuilder(PathOne.end())           //Move to the wobble goal from ONE
                .splineToSplineHeading(new Pose2d(-5,-14),Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-30, -15, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Trajectory BackfromTwo = drive.trajectoryBuilder(PathTwo.end())          //Move to the wobble goal from TWO
                .splineToSplineHeading(new Pose2d(-5,-14),Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-30, -15, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Trajectory Picking = drive.trajectoryBuilder(BackfromZero.end())            //Move the robot to make contact with second wobble goal
                .lineToConstantHeading(new Vector2d(-32,-22))
                .build();
        
        Trajectory PathZeroB = drive.trajectoryBuilder(Picking.end())             //ZERO path picks off when the first picking ends
                .splineTo(new Vector2d(8,-42), Math.toRadians(90))                //Move to first square v
                .build();

        Trajectory PathOneB = drive.trajectoryBuilder(Picking.end())             //ONE path picks off when the first picking ends
                .splineTo(new Vector2d(25,-42), Math.toRadians(179))                //Move to second square
                .build();

        Trajectory PathTwoB = drive.trajectoryBuilder(Picking.end())             //TWO path picks off when the first picking ends
                .splineTo(new Vector2d(55,-42), Math.toRadians(90))                //Move to third square
                .build();

        Trajectory toLineZero = drive.trajectoryBuilder(PathZeroB.end())           //Move to the line from ZERO
                .splineToLinearHeading(new Pose2d(8, -30, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Trajectory toLineOne = drive.trajectoryBuilder(PathOneB.end())           //Move to the line from ONE
                .splineToLinearHeading(new Pose2d(8, -30, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Trajectory toLineTwo = drive.trajectoryBuilder(PathTwoB.end())          //Move to the line from TWO
                .splineToLinearHeading(new Pose2d(8, -30, Math.toRadians(0)), Math.toRadians(0))
                .build();


        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode, servos have been set");
        telemetry.update();
        GripperA.setPosition(Range.clip(0,0,1));
        GripperB.setPosition(Range.clip(0.5, 0, 1));




        waitForStart();


        // wait for start button.

        waitForStart();


        //Start of move in front of rings

        drive.followTrajectory(BeforeShooting);

        shootRingsPhaseA();

        drive.followTrajectory(DuringShooting);

        shootRingsPhaseB();

        drive.followTrajectory(MovetoRings);

        //End of move in front of rings

        /**Tensor flow start: */
        long j = 0;
        int Path = 0;                                          //Base value, end path should be =! 0 if a path is detected

        while ( j < 2000000) {                //Makes sure that a proper path is returned, if no new value exists, the robot will exit after a set ammount of elapsed time (Determined by # of iterations) T

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

                    telemetry.addData("Count: 2,000,000>", j);
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


        if (Path==0){

            drive.followTrajectory(PathZero);

            dropGoal();

            drive.followTrajectory(BackfromZero);


        } else if (Path == 1) {

            drive.followTrajectory(PathOne);

            dropGoal();

            drive.followTrajectory(BackfromOne);

        } else if (Path == 2) {

            drive.followTrajectory(PathTwo);

            dropGoal();

            drive.followTrajectory(BackfromTwo);

        }

        pickGoalPhaseA();

        drive.followTrajectory(Picking);

        pickGoalPhaseB();


        if (Path==0){

            drive.followTrajectory(PathZeroB);

            dropGoal();

            drive.followTrajectory(toLineZero);


        } else if (Path == 1) {

            drive.followTrajectory(PathOneB);

            dropGoal();

            drive.followTrajectory(toLineOne);

        } else if (Path == 2) {

            drive.followTrajectory(PathTwoB);

            dropGoal();

            drive.followTrajectory(toLineTwo);

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

    private void dropGoal(){
        sleep(400);
        ArmBase.setPower(-0.4);                                                       //Drops off the wobble goal
        sleep(1500);
        ArmBase.setPower(0);
        GripperA.setPosition(Range.clip(0.5, 0, 1));
        GripperB.setPosition(Range.clip(0,0,1));

        sleep(1000);
        ArmBase.setPower(0.4);
        sleep(1000);
    }

    private void pickGoalPhaseA(){
        ArmBase.setPower(-0.4);                                                       //Brings Arm down
        sleep(1500);
        ArmBase.setPower(0);
    }

    private void pickGoalPhaseB(){
        GripperA.setPosition(Range.clip(0,0,1));                  // Grabs and lifts arm
        GripperB.setPosition(Range.clip(0.5, 0, 1));

        sleep(1000);
        ArmBase.setPower(0.4);
        sleep(500);
    }

    private void shootRingsPhaseA(){

        LeftShooter.setPower(0.95);                                                //Sets motor powers for shooting
        RightShooter.setPower(-0.95);
        sleep(500);
        Intake.setPower(-1);

    }

    private void shootRingsPhaseB(){
        LeftShooter.setPower(0);                                               //Shuts down motors after shooting
        RightShooter.setPower(0);
        Intake.setPower(0);
    }




}
