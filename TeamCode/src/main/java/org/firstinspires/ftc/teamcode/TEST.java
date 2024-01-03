package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "TEST (Blocks to Java)")
public class TEST extends LinearOpMode {

    private DcMotor FrontLeftMotor;
    private DcMotor BackLeftMotor;
    private DcMotor ArmMotor;
    private DcMotor ClawMotor;
    private Servo LeftServo;
    private Servo RightServo;
    private DcMotor FrontRightMotor;
    private DcMotor BackRightMotor;
    private IMU imu;

    int tickPerInch;
    int OnLine;
    List<AprilTagDetection> myAprilTagDetections;
    VisionPortal myVisionPortal;
    int stepsCount;
    float x;
    AprilTagProcessor myAprilTagProcessor;
    TfodProcessor myTfodProcessor;
    double tickPerInchStrafe;
    AprilTagDetection myAprilTagDetection;
    boolean USE_WEBCAM;

    /**
     * Describe this function...
     */
    private void ActionOnline3() {
        Forward((int) (26.5 * tickPerInch - stepsCount * tickPerInch), 0.5);
        sleep(300);
        Turn(90, 1);
        sleep(200);
        Forward(2 * tickPerInch, 0.3);
        // ****************Drop First Pixel***********
        OpenClawSlightly();
        sleep(300);
        MoveWrist(-200, 0.1);
        CloseClaw();
        sleep(200);
        MoveWrist(0, 0.8);
        sleep(500);
        MoveArm2(8000, 1);
        MoveWrist(-1131, 0.8);
        MoveArm2(1800, 1);
        Forward(-23 * tickPerInch, 0.4);
        sleep(200);
        Strafe(7 * tickPerInchStrafe, 0.3);
        sleep(300);
        Forward(-5 * tickPerInch, 0.3);
        // ****************Drop Second Pixel***********
        OpenClawSlightly2();
        sleep(200);
        Forward(1 * tickPerInch, 0.2);
        Strafe(8 * tickPerInchStrafe, 0.4);
        sleep(200);
        MoveArm2(-3500, 0.8);
        Strafe(8 * tickPerInchStrafe, 0.5);
        MoveWrist(0, 0.3);
        MoveArm2(-4000, 0.8);
        Forward(-10 * tickPerInch, 0.4);
        sleep(200);
    }

    /**
     * Describe this function...
     */
    private void ActionOnline1() {
        Strafe(6.5 * tickPerInchStrafe, 0.4);
        sleep(200);
        Forward(23 * tickPerInch - stepsCount * tickPerInch, 0.4);
        sleep(300);
        OpenClawSlightly();
        MoveWrist(-200, 0.2);
        sleep(200);
        CloseClaw();
        sleep(100);
        MoveWrist(0, 0);
        Forward(-5 * tickPerInch, 0.2);
        Turn(90, 1);
        Strafe(3 * tickPerInchStrafe, 0.3);
        sleep(200);
        MoveArm(8000, 1);
        MoveWrist(-1131, 0.9);
        MoveArm2(1800, 1);
        Forward((int) (-18.5 * tickPerInch), 0.3);
        sleep(200);
        OpenClawSlightly2();
        Forward(1 * tickPerInch, 0.3);
        sleep(200);
        MoveArm2(-1400, 1);
        Strafe(25 * tickPerInchStrafe, 0.5);
        MoveWrist(0, 0.8);
        Forward(-13 * tickPerInch, 0.4);
        MoveArm2(-7100, 1);
        sleep(200);
    }

    /**
     * Describe this function...
     */
    private void ActionOnline2() {
        telemetry.addData("***** OpenClaw On line#:", OnLine);
        telemetry.update();
        Forward(25 * tickPerInch - stepsCount * tickPerInch, 0.5);
        sleep(100);
        // ****************Drop First Pixel***********
        OpenClawSlightly();
        sleep(200);
        MoveWrist(-200, 0.2);
        CloseClaw();
        MoveWrist(0, 0.3);
        sleep(200);
        Forward(-4 * tickPerInch, 0.3);
        sleep(100);
        Turn(90, 1);
        sleep(200);
        MoveArm2(8000, 1);
        MoveWrist(-1151, 0.9);
        MoveArm2(1800, 1);
        Strafe(6 * tickPerInchStrafe, 0.3);
        sleep(300);
        Forward(-24 * tickPerInch, 0.4);
        sleep(200);
        Forward(-3 * tickPerInch, 0.2);
        // ****************Drop Second Pixel***********
        sleep(200);
        OpenClawSlightly2();
        sleep(200);
        Forward(1 * tickPerInch, 0.3);
        MoveArm2(-1000, 0.8);
        Strafe(20 * tickPerInchStrafe, 0.4);
        MoveArm2(-3500, 0.8);
        MoveWrist(0, 0.3);
        Forward(-12 * tickPerInch, 0.2);
        sleep(200);
        MoveArm2(-4000, 0.8);
        sleep(200);
    }

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        ClawMotor = hardwareMap.get(DcMotor.class, "ClawMotor");
        LeftServo = hardwareMap.get(Servo.class, "LeftServo");
        RightServo = hardwareMap.get(Servo.class, "RightServo");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        BackRightMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");
        imu = hardwareMap.get(IMU.class, "imu");

        OnLine = -1;
        tickPerInch = 46;
        tickPerInchStrafe = 52.57;
        // Put initialization blocks here.
        USE_WEBCAM = true;
        CloseClaw();
        stepsCount = 0;
        initVision();
        FrontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        BackLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ClawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        if (opModeIsActive()) {
            MoveWrist(-500, 0.3);
            // Get the state of the camera.
            if (("" + myVisionPortal.getCameraState()).equals("ERROR")) {
                OnLine = 0;
                telemetry.addData("***Camera Error****", 0);
                telemetry.update();
                sleep(1000);
            } else {
                SearchObject();
                sleep(200);
                for (int count = 0; count < 3; count++) {
                    if (OnLine == -1) {
                        Forward(1 * tickPerInch, 0.2);
                        stepsCount = stepsCount + 1;
                        sleep(200);
                        SearchObject();
                    } else {
                        break;
                    }
                }
                if (OnLine == -1) {
                    OnLine = 3;
                }
            }
            if (OnLine == 0) {
                ActionOnline3();
            } else if (OnLine == 1) {
                ActionOnline1();
            } else if (OnLine == 2) {
                ActionOnline2();
            } else if (OnLine == 3) {
                ActionOnline3();
                sleep(10000);
                stepsCount = 0;
            } else {
            }
        }
    }

    /**
     * Describe this function...
     */
    private void OpenClawSlightly() {
        LeftServo.setPosition(0.09);
        RightServo.setPosition(0.38);
    }

    /**
     * Describe this function...
     */
    private void OpenClaw() {
        LeftServo.setPosition(0.2);
        RightServo.setPosition(0.2);
        sleep(200);
    }

    /**
     * Describe this function...
     */
    private void CloseClaw() {
        LeftServo.setPosition(0.01);
        RightServo.setPosition(0.4);
    }

    /**
     * Describe this function...
     */
    private void OpenClawSlightly2() {
        LeftServo.setPosition(0.12);
        RightServo.setPosition(0.33);
        sleep(200);
    }

    /**
     * Initialize TensorFlow Object Detection.
     */
    private void initVision() {
        TfodProcessor.Builder myTfodProcessorBuilder;
        AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;

        // First, create a TfodProcessor.Builder.
        myTfodProcessorBuilder = new TfodProcessor.Builder();
        // Set the name of the file where the model can be found.
        myTfodProcessorBuilder.setModelFileName("BlueTeamProp.tflite");
        // Set the full ordered list of labels the model is trained to recognize.
        myTfodProcessorBuilder.setModelLabels(JavaUtil.createListWith("BLP"));
        // Set the aspect ratio for the images used when the model was created.
        myTfodProcessorBuilder.setModelAspectRatio(16 / 9);
        // Create a TfodProcessor by calling build.
        myTfodProcessor = myTfodProcessorBuilder.build();
        // Next, create a VisionPortal.Builder and set attributes related to the camera.
        // Add myTfodProcessor to the VisionPortal.Builder.
        // Create a VisionPortal by calling build.
        // Set the minimum confidence at which to keep recognitions.
        myTfodProcessor.setMinResultConfidence((float) 0.8);
        // Create a VisionPortal, with the specified webcam name and
        // TensorFlow Object Detection processor, and assign it to a variable.
        myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), myTfodProcessor);
    }

    /**
     * Describe this function...
     */
    private void MoveArm(int Ticks, double Power) {
        //
        // set motor mode to run to position
        ArmMotor.setTargetPosition(Ticks);
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotor.setPower(Power);
        while (!!ArmMotor.isBusy()) {
            telemetry.update();
        }
    }

    /**
     * Describe this function...
     */
    private void SearchAprilTag(
            // TODO: Enter the type for argument named ID
            int ID) {
        int aprilTag_X;
        int aprilTag_Y;
        int aprilTag_Yaw;
        boolean aprilTag_IsFound;

        aprilTag_X = 0;
        aprilTag_Y = 0;
        aprilTag_Yaw = 0;
        aprilTag_IsFound = false;
        // Get a list of AprilTag detections.
        myAprilTagDetections = myAprilTagProcessor.getDetections();
        telemetry.addData("Searching", myVisionPortal);
        // Push telemetry to the Driver Station.
        telemetry.update();
        // Iterate through list and call a function to display info for each recognized AprilTag.
        for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
            myAprilTagDetection = myAprilTagDetection_item;
            // Display info about the detection.
            if (myAprilTagDetection.metadata != null && myAprilTagDetection.id == ID) {
                aprilTag_IsFound = true;
                aprilTag_X = Integer.parseInt(JavaUtil.formatNumber(myAprilTagDetection.ftcPose.x, 6, 1));
                aprilTag_Y = Integer.parseInt(JavaUtil.formatNumber(myAprilTagDetection.ftcPose.y, 6, 1));
                aprilTag_Yaw = Integer.parseInt(JavaUtil.formatNumber(myAprilTagDetection.ftcPose.yaw, 6, 1));
                telemetry.addData("X - Y - Yaw", "" + aprilTag_X + aprilTag_Y + aprilTag_Yaw);
                // Push telemetry to the Driver Station.
                telemetry.update();
                // Share the CPU.
                sleep(1000);
            } else {
            }
        }
    }

    /**
     * Describe this function...
     */
    private void Forward(int Ticks, double Power) {
        // Forward2; converted to DumE by Julia
        FrontLeftMotor.setTargetPosition(FrontLeftMotor.getCurrentPosition() + Ticks);
        FrontRightMotor.setTargetPosition(FrontRightMotor.getCurrentPosition() + Ticks);
        BackLeftMotor.setTargetPosition(BackLeftMotor.getCurrentPosition() + Ticks);
        BackRightMotor.setTargetPosition(BackRightMotor.getCurrentPosition() + Ticks);
        // set motor mode to run to position
        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeftMotor.setPower(Power);
        BackRightMotor.setPower(Power);
        BackLeftMotor.setPower(Power);
        FrontRightMotor.setPower(Power);
        while (BackLeftMotor.isBusy()) {
        }
    }

    /**
     * Describe this function...
     */
    private void MoveWrist(int Ticks, double Power) {
        //
        ClawMotor.setTargetPosition(Ticks);
        // set motor mode to run to position
        ClawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ClawMotor.setPower(Power);
        while (!!ClawMotor.isBusy()) {
            telemetry.update();
        }
    }

    /**
     * Describe this function...
     */
    private void MoveDiag(double pwrLfRb, double pwrRfLb, long runTime) {
        FrontLeftMotor.setPower(pwrLfRb);
        BackLeftMotor.setPower(pwrRfLb);
        FrontRightMotor.setPower(pwrRfLb);
        BackRightMotor.setPower(pwrLfRb);
        sleep(runTime);
        FrontLeftMotor.setPower(0);
        BackLeftMotor.setPower(0);
        FrontRightMotor.setPower(0);
        BackRightMotor.setPower(0);
    }

    /**
     * Describe this function...
     */
    private void Strafe(double Ticks, double Power) {
        // Strafe; converted to DumE by Julia
        FrontLeftMotor.setTargetPosition((int) (FrontLeftMotor.getCurrentPosition() - Ticks));
        FrontRightMotor.setTargetPosition((int) (FrontRightMotor.getCurrentPosition() + Ticks));
        BackLeftMotor.setTargetPosition((int) (BackLeftMotor.getCurrentPosition() + Ticks));
        BackRightMotor.setTargetPosition((int) (BackRightMotor.getCurrentPosition() - Ticks));
        // set motor mode to run to position
        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Engage motors at 30% power
        FrontLeftMotor.setPower(Power);
        BackRightMotor.setPower(Power);
        BackLeftMotor.setPower(-Power);
        FrontRightMotor.setPower(-Power);
        while (BackLeftMotor.isBusy()) {
        }
    }

    /**
     * Describe this function...
     */
    private void MoveArm2(int Ticks, double Power) {
        // set motor mode to run to position
        ArmMotor.setTargetPosition(ArmMotor.getCurrentPosition() + Ticks);
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotor.setPower(Power);
        while (!!ArmMotor.isBusy()) {
            telemetry.update();
        }
    }

    /**
     * Describe this function...
     */
    private void SearchObject() {
        List<Recognition> myTfodRecognitions;
        Recognition myTfodRecognition;
        float y;

        for (int count2 = 0; count2 < 20; count2++) {
            // Get a list of recognitions from TFOD.
            myTfodRecognitions = myTfodProcessor.getRecognitions();
            // Share the CPU.
            sleep(20);
            // Iterate through list and call a function to display info for each recognized object.
            for (Recognition myTfodRecognition_item : myTfodRecognitions) {
                myTfodRecognition = myTfodRecognition_item;
                // Display info about the recognition.
                // Display label and confidence.
                x = (myTfodRecognition.getLeft() + myTfodRecognition.getRight()) / 2;
                y = (myTfodRecognition.getTop() + myTfodRecognition.getBottom()) / 2;
                if (x < 360) {
                    OnLine = 1;
                } else if (x >= 360 && x < 700) {
                    OnLine = 2;
                } else {
                    OnLine = 3;
                }
            }
            // Share the CPU.
            sleep(20);
            if (OnLine != 0) {
                break;
            }
        }
        telemetry.update();
    }

    /**
     * Describe this function...
     */
    private void MakeTurn(double Ticks, double direction) {
        // Turn***
        FrontLeftMotor.setTargetPosition((int) (FrontLeftMotor.getCurrentPosition() + direction * Ticks));
        BackLeftMotor.setTargetPosition((int) (BackLeftMotor.getCurrentPosition() + direction * Ticks));
        FrontRightMotor.setTargetPosition((int) (FrontRightMotor.getCurrentPosition() + -1 * direction * Ticks));
        BackRightMotor.setTargetPosition((int) (BackRightMotor.getCurrentPosition() + -1 * direction * Ticks));
        // set motor mode to run to position
        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeftMotor.setPower(0.3);
        BackLeftMotor.setPower(0.3);
        FrontRightMotor.setPower(0.3);
        BackRightMotor.setPower(0.3);
        while (BackLeftMotor.isBusy()) {
            sleep(10);
        }
    }

    /**
     * Describe this function...
     */
    private void ActionOnline0() {
        Strafe(40 * tickPerInchStrafe, 0.3);
        sleep(500);
    }

    /**
     * Describe this function...
     */
    /*private void DetectLine() {
        double TicksPerTile;

        while (0 < 200) {
            telemetry.addData("Current Hue", 123);
            telemetry.update();
            sleep(1000);
            Forward((int) (0.05 * TicksPerTile), 0.1);
        }
    }
*/
    /**
     * Describe this function...
     */
    private void Turn(int degree, int direction) {
        YawPitchRollAngles orientation;
        double CurrentYaw;
        double TargetYaw;
        double YawDiff;

        // Turn***
        orientation = imu.getRobotYawPitchRollAngles();
        CurrentYaw = orientation.getYaw(AngleUnit.DEGREES);
        TargetYaw = CurrentYaw + degree * -1 * direction;
        if (degree > 0) {
            MakeTurn((degree - 0) * 10.95, direction);
        } else {
        }
    }

    /**
     * Describe this function...
     */
    private void Move(double leftPower, double rightPower, long runTime) {
        telemetry.addData("Move - left power =", leftPower);
        telemetry.update();
        FrontLeftMotor.setPower(leftPower);
        BackLeftMotor.setPower(leftPower);
        FrontRightMotor.setPower(rightPower);
        BackRightMotor.setPower(rightPower);
        sleep(runTime);
        FrontLeftMotor.setPower(0);
        BackLeftMotor.setPower(0);
        FrontRightMotor.setPower(0);
        BackRightMotor.setPower(0);
    }

    /**
     * Initialize AprilTag Detection.
     */
    private void initAprilTag() {
        // First, create an AprilTagProcessor.
        myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        // Next, create a VisionPortal.
        if (USE_WEBCAM) {
            myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), myAprilTagProcessor);
        } else {
            myVisionPortal = VisionPortal.easyCreateWithDefaults(BuiltinCameraDirection.BACK, myAprilTagProcessor);
        }
    }

    /**
     * Initialize TensorFlow Object Detection.
     */
    private void initTfod() {
        // First, create a TfodProcessor.
        myTfodProcessor = TfodProcessor.easyCreateWithDefaults();
        // Set the minimum confidence at which to keep recognitions.
        myTfodProcessor.setMinResultConfidence((float) 0.6);
        // Indicate that only the zoomed center area of each image will be passed to
        // the TensorFlow object detector. For no zooming, set magnification to 1.0.
        myTfodProcessor.setZoom(1);
        // Next, create a VisionPortal.
        if (USE_WEBCAM) {
            myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 2"), myTfodProcessor);
        } else {
            myVisionPortal = VisionPortal.easyCreateWithDefaults(BuiltinCameraDirection.BACK, myTfodProcessor);
        }
    }

    /**
     * Describe this function...
     */
    private void DropPixel1() {
        sleep(2000);
    }
}