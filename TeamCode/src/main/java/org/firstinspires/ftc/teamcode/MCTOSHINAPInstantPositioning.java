package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(group = "Driver")
public class MCTOSHINAPInstantPositioning extends LinearOpMode {
    // Define 2 states, drive control or automatic control
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    /** The coordinates we want the bot to automatically go to when we press the A button, find the point we want to shoot at */
    Vector2d targetAVector = new Vector2d(0, -36);

    // The heading we want the bot to end on for targetA
    double targetAHeading = Math.toRadians(0);

    Pose2d ShootPose = new Pose2d(0,-36, Math.toRadians(0));



    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize custom cancelable SampleMecanumDrive class
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        DcMotor  LeftShooter, RightShooter, ArmBase, Intake;
        //TouchSensor Touch;
        Servo Gripper;
        double ConstRes, IntakePower;
        double LaunchPower;
        boolean LeftBumper, buttonX, RightBumper;
        boolean AutoOn = false;
        boolean AutoSwitch = false;
        boolean AutoIn = false;
        boolean PreviousBumper = false;
        boolean PreviousbuttonX = false;
        boolean PreviousIntake = false;
        float GripStregnth = 0;
        double ArmPower = 0;

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        LeftShooter = hardwareMap.dcMotor.get("LeftShooter");
        RightShooter = hardwareMap.dcMotor.get("RightShooter");
        Intake = hardwareMap.dcMotor.get("Intake");
        ArmBase = hardwareMap.dcMotor.get("ArmBase");
        Gripper = hardwareMap.servo.get("Gripper");
        ArmBase.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Update the drive class
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("mode", currentMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            // We follow different logic based on whether we are in manual driver control or switch
            // control to the automatic mode
            switch (currentMode) {
                case DRIVER_CONTROL:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );

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
                        ArmPower = -0.6;
                    if (gamepad1.dpad_up)
                        ArmPower = 0.4;



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
                        LaunchPower = -0.4;
                    } else {
                        IntakePower = 0;
                    }

                    if(gamepad1.right_trigger != 0)
                        IntakePower =  -gamepad1.right_trigger;

                    PreviousIntake = RightBumper;

                    //End of Intake Code

                    //Start of PreShoot Sequence
                    if (gamepad1.b==true){
                        LaunchPower = -0.8;
                        IntakePower=0.4;
                    }
                    //End of PreShoot Sequence


                    //Power Assignment
                    RightShooter.setPower(-LaunchPower);
                    LeftShooter.setPower(LaunchPower);
                    Intake.setPower(IntakePower);
                    ArmBase.setPower(ArmPower);


                    if(gamepad1.y) {
                        drive.setPoseEstimate(ShootPose);
                    }

                    if (gamepad1.a && poseEstimate != ShootPose) {
                        // If the A button is pressed on gamepad1, we generate a splineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .splineTo(targetAVector, targetAHeading)
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }
                    break;
                case AUTOMATIC_CONTROL:
                    // If b is pressed, we break out of the automatic following
                    if (gamepad1.b) {
                        drive.cancelFollowing();
                        currentMode = Mode.DRIVER_CONTROL;
                    }

                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }
        }
    }
}