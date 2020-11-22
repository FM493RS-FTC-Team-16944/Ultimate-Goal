package org.firstinspires.ftc.teamcode;

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

import java.util.List;


@Autonomous(name="WobGoal", group="Autonomous")

public class WobbleGoal extends LinearOpMode {

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
    DcMotor FrontLeftDrive, FrontRightDrive, BackLeftDrive, BackRightDrive, LeftShooter, RightShooter, ArmBase;
    Servo Gripper;
    long Motion;
    double   FLPower, FRPower, BLPower, BRPower,xValue, yValue;
    //end of motion init

    @Override
    public void runOpMode() throws InterruptedException
    {
        FrontLeftDrive = hardwareMap.dcMotor.get("FrontLeftDrive");
        FrontRightDrive = hardwareMap.dcMotor.get("FrontRightDrive");
        BackLeftDrive = hardwareMap.dcMotor.get("BackLeftDrive");
        BackRightDrive = hardwareMap.dcMotor.get("BackRightDrive");
        LeftShooter = hardwareMap.dcMotor.get("LeftShooter");
        RightShooter = hardwareMap.dcMotor.get("RightShooter");
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

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();


        // wait for start button.

        waitForStart();



        //Pick Up Goal
        Gripper.setPosition(Range.clip(0.5, 0, 1));
        sleep(2000);
        ArmBase.setPower(0.3);
        sleep(850);
        ArmBase.setPower(-0.1);
        sleep(750);
        Gripper.setPosition(Range.clip(0,0,1));
        sleep(2000);
        ArmBase.setPower(-0.4);
        Gripper.setPosition(Range.clip(0,0,1));
        sleep(800);
        ArmBase.setPower(-0.2);
        Gripper.setPosition(Range.clip(0,0,1));
        sleep(400);
        ArmBase.setPower(-0.1);
        Gripper.setPosition(Range.clip(0,0,1));



        MecanumDirectionalFunction r = new MecanumDirectionalFunction();
        r.Calculation(0, 0, 0.4);                              //Calculate
        FrontLeftDrive.setPower(r.GetFrontLeftPower());                              //Set Motor powers
        BackLeftDrive.setPower(r.GetBackLeftPower());
        FrontRightDrive.setPower(r.GetFrontRightPower());
        BackRightDrive.setPower(r.GetBackRightPower());
        sleep(400);

        MecanumDirectionalFunction x = new MecanumDirectionalFunction();
         x.Calculation(1, 0, 0);                              //Calculate
        FrontLeftDrive.setPower(x.GetFrontLeftPower());                              //Set Motor powers
        BackLeftDrive.setPower(x.GetBackLeftPower());
        FrontRightDrive.setPower(x.GetFrontRightPower());
        BackRightDrive.setPower(x.GetBackRightPower());
        sleep(750);

        MecanumDirectionalFunction O = new MecanumDirectionalFunction();
        O.Calculation(0, 0, 0);                              //Calculate
        FrontLeftDrive.setPower(O.GetFrontLeftPower());                              //Set Motor powers
        BackLeftDrive.setPower(O.GetBackLeftPower());
        FrontRightDrive.setPower(O.GetFrontRightPower());
        BackRightDrive.setPower(O.GetBackRightPower());
        sleep(400);







        /**Tensor flow start: */
        long j = 0;
        int Path = 0;                                          //Base value, end path should be =! 0 if a path is detected
        while ( j < 5000000) {                //Makes sure that a proper path is returned, if no new value exists, the robot will exit after a set ammount of elapsed time (Determined by # of iterations)

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

                        if (recognition.getLabel() == "Single") {               //Takes the string and determines what type of object it is
                            Path = 1;
                            break;                                              //Exits the loop early if one ring is detected
                        } else if (recognition.getLabel() == "Quad") {
                            Path = 2;
                            break;                                             //Exits the loop once the stack of rings is detected
                        }
                    }

                    telemetry.addData("Count: 5,000,000>", j);
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

        } else if (Path == 1) {

        } else if (Path == 2) {

        }
        sleep(10000);











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

