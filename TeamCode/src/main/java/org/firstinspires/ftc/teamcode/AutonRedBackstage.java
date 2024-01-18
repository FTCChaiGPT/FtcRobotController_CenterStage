package org.firstinspires.ftc.teamcode;

import android.widget.Switch;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//imports For AprilTAG
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

//For Identifying Team Prop
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;

//For drive
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@Autonomous(name="AutonRedBackstage", group = "Auto")
public class AutonRedBackstage extends LinearOpMode{
    private DcMotor leftFront_motor;
    private DcMotor leftBack_motor;
    private DcMotor rightFront_motor;
    private DcMotor rightBack_motor;
    private DcMotor intake_motor;
    private Servo pusher_servo;
    private Servo gate_servo;
    private Servo scooper_servo;
    public enum PropPostition {
        LEFT, CENTER, RIGHT, UNKNOWN

    }
    public PropPostition spikePosition;
    private static final double COUNTS_PER_MOTOR_REV = 750; //Number of encoder counts per motor revolution (1440)
    private static final double WHEEL_DIAMETER_INCHES = 3.75;
    private static final double GEAR_RATIO = 1.0;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * GEAR_RATIO)/(WHEEL_DIAMETER_INCHES * Math.PI);
    private static final double ROBOT_WIDTH_INCHES = 18.9; // The distance between the wheels on opposite sides (diagonal)
    private double motorPowerValue = 0.5;
    private double intakePowerValue = 0.5;
    private int sleepSecond = 750;
    private boolean purplePixelDropped = false;
    private boolean firstContourDetected = false;
    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;
    public int AprilTagDetected = 0;
    public int WantedAprilTag;
    public boolean SearchAprilTAG = false;
    private ElapsedTime runtime = new ElapsedTime(); //timer, used to remove sleeps

    OpenCvCamera webcam;
    SamplePipeline pipeline;
    private boolean startProcessing = false; // Flag to control when to start processing

    @Override
    public void runOpMode() {
        DriveInitialization();
        CameraInitialization();
        //AprilTAGInitialization();
        waitForStart();

        pipeline.startProcessing = true;
        firstContourDetected = false;

        while (opModeIsActive() && !firstContourDetected) {
            // Your autonomous or teleop code here
            spikePosition = pipeline.getLastDetectedPosition();

            if(spikePosition != PropPostition.UNKNOWN) {
                firstContourDetected = true;
                telemetry.addLine("Prop Detected");
                telemetry.addData("Position", spikePosition);
                webcam.stopStreaming();
                //sleep(10000);
            }

            // Telemetry (optional)
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.update();

        }

        telemetry.addLine("Ready to drop  purple");

        DropPurplePixel(spikePosition);
            //NavigateToBackstage(spikePosition);
            //DropYellowPixel(spikePosition);
            //webcam.stopStreaming(); // Stop streaming when the op mode is no longer active




    }

    public int DetectAprilTag(boolean SearchAprilTAG) {
        if (!SearchAprilTAG){
            return 0;
        }
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

                return (detection.id);

            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
        return 0;
    }
    class SamplePipeline extends OpenCvPipeline {

        private static final double LEFT_REGION = 0.33;
        private static final double RIGHT_REGION = 0.66;
        private boolean startProcessing = false;

        public PropPostition lastDetectedPosition = PropPostition.UNKNOWN;

        @Override
        public Mat processFrame(Mat input) {
            if (!startProcessing) {
                return input;
            }
            lastDetectedPosition = PropPostition.UNKNOWN;

            // Process the frame here. For example, detect objects or patterns.
            Mat hsvImage = new Mat();//
            Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_RGB2HSV);

            Scalar lowerRed = new Scalar(0, 150, 50);
            Scalar upperRed = new Scalar(20, 255, 255);
            Mat blueMask = new Mat();
            Core.inRange(hsvImage, lowerRed, upperRed, blueMask);

            List<org.opencv.core.MatOfPoint> contours = new ArrayList<>();
            Mat hierachy = new Mat();
            Imgproc.findContours(blueMask, contours, hierachy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            for (org.opencv.core.MatOfPoint contour : contours) {
                if (Imgproc.contourArea(contour) > 2500) {
                    Rect boundingRect = Imgproc.boundingRect(contour);
                    double centerx = boundingRect.x + (boundingRect.width / 2.0);

                    if (centerx < input.cols() * LEFT_REGION) {
                        lastDetectedPosition = PropPostition.LEFT;
                    } else if (centerx > input.cols() * RIGHT_REGION) {
                        lastDetectedPosition = PropPostition.RIGHT;
                    } else {
                        lastDetectedPosition = PropPostition.CENTER;
                    }

                    break; //to detect only 1 item that satisfies the size of the contour
                }
            }
            hsvImage.release();
            blueMask.release();
            hierachy.release();

            return input;
        }
        public PropPostition getLastDetectedPosition(){
            return lastDetectedPosition;
        }
    }

    private void DropPurplePixel(PropPostition spikePosition){
        //till reverse intake
        PropPostition spikePosition1 = spikePosition;
        switch (spikePosition1) {
            case LEFT:
                //left logic
                forward(27, motorPowerValue);
                turn_Left(90,motorPowerValue);
                forward(10,motorPowerValue);
                intake_motor.setPower(intakePowerValue);
                break;

            case CENTER:
                //center logic
                forward(21,motorPowerValue);
                intake_motor.setPower(intakePowerValue);
                break;

            case RIGHT:
                //right logic
                forward(27,motorPowerValue);
                turn_Right(90,motorPowerValue);
                forward(10,motorPowerValue);
                intake_motor.setPower(intakePowerValue);
                break;

            case UNKNOWN:
                //unknown logic
                telemetry.addLine("Unknown, so trial and error on center logic");
                //forward(40,motorPowerValue);
                //intake_motor.setPower(intakePowerValue);
                break;

        }
    }
    private void NavigateToBackstage(PropPostition spikePosition) {
        PropPostition spikePosition1 = spikePosition;
        switch (spikePosition) {
            case LEFT:
                //left logic
                backward(5,motorPowerValue);
                strafe_Left(6,motorPowerValue);
                forward(35,motorPowerValue);
                break;

            case CENTER:
                //center logic
                backward(10,motorPowerValue);
                turn_Left(90,motorPowerValue);
                forward(48,motorPowerValue);
                break;

            case RIGHT:
                //right logic
                backward(3,motorPowerValue);
                turn_Left(180,motorPowerValue);
                forward(48,motorPowerValue);
                break;

            case UNKNOWN:
                //unknown logic
                //since unknown, go for center logic
                telemetry.addLine("Unknown, so trial and error on center logic");
                backward(10,motorPowerValue);
                turn_Left(90,motorPowerValue);
                forward(48,motorPowerValue);
                break;
        }
    }
    private void DropYellowPixel(PropPostition spikePosition){
        AprilTagDetected = 0;
        telemetry.addData("SearchAprilTAG", SearchAprilTAG);
        SearchAprilTAG = false;

        if (runtime.milliseconds() > 5000){
            SearchAprilTAG = true;
        }

        AprilTagDetected = DetectAprilTag(SearchAprilTAG);
        telemetry.addData("SearchAprilTAG", SearchAprilTAG);
        telemetry.addData("AprilTagDetected", AprilTagDetected);

        PropPostition spikePosition1 = spikePosition;
        switch (spikePosition) {
            case LEFT:
                telemetry.addData("AprilTagDetected", AprilTagDetected);
                telemetry.addLine("Detected 4");
                if (AprilTagDetected == WantedAprilTag) {
                    telemetry.addLine("Drop Pixel on Backdrop");
                    visionPortal.close();
                }
                //code for arm placing on backdrop here
                strafe_Left(12, motorPowerValue);
                break;

            case CENTER:
                telemetry.addData("AprilTagDetected", AprilTagDetected);
                telemetry.addLine("Detected 5");
                if (AprilTagDetected == WantedAprilTag) {
                    telemetry.addLine("Drop Pixel on Backdrop");
                    visionPortal.close();
                }
                //code for arm placing on backdrop here
                strafe_Left(18, motorPowerValue);
                break;

            case RIGHT:
                telemetry.addData("AprilTagDetected", AprilTagDetected);
                telemetry.addLine("Detected 6");
                if (AprilTagDetected == WantedAprilTag) {
                    telemetry.addLine("Drop Pixel on Backdrop");
                    visionPortal.close();
                }
                //code for arm placing on backdrop here
                strafe_Left(24, motorPowerValue);
                break;

            case UNKNOWN:
                telemetry.addData("AprilTagDetected", AprilTagDetected);
                telemetry.addLine("Unable to detect April Tag");
                break;
        }
        telemetry.update();
    }



    //All initialization codes are below...

    public void DriveInitialization(){
        leftFront_motor = hardwareMap.get(DcMotor.class, "left_front");
        leftBack_motor = hardwareMap.get(DcMotor.class, "left_back");
        rightFront_motor = hardwareMap.get(DcMotor.class, "right_front");
        rightBack_motor = hardwareMap.get(DcMotor.class, "right_back");
        intake_motor = hardwareMap.get(DcMotor.class, "intake");
        pusher_servo = hardwareMap.get(Servo.class, "pusher");
        gate_servo = hardwareMap.get(Servo.class, "gate");
        scooper_servo = hardwareMap.get(Servo.class, "front");

        leftFront_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront_motor.setDirection(DcMotor.Direction.REVERSE);
        rightBack_motor.setDirection(DcMotor.Direction.REVERSE);
        intake_motor.setDirection(DcMotor.Direction.FORWARD);

        leftFront_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Motor & Servo init. completed");
    }
    private void CameraInitialization(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new SamplePipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                telemetry.addLine("Camera is Open.. streaming is LIVE.");
                // Do not start streaming yet
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Camera has error...");
                // Handle camera open error
            }


        });
    }
    public void AprilTAGInitialization() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

    }

    //Drive codes and functions are below...

    public void slowDownAtEnd(double p) {
        while (opModeIsActive() && leftFront_motor.isBusy()) { //Accurate measures
            double remainingDistance = Math.abs(leftFront_motor.getTargetPosition() - leftFront_motor.getCurrentPosition());
            // Adjust this threshold as needed.
            double rampDownThreshold = 600;  // For example, start ramping down when 300 encoder counts away from target.

            if (remainingDistance < rampDownThreshold) {
                double rampedSpeed = 0.5 * (p * (remainingDistance / rampDownThreshold));
                rampedSpeed = Math.max(rampedSpeed, 0.1); // Ensure you have a minimum speed to prevent the robot from stalling.
                setMotorPower(rampedSpeed);
            }
        }
    }

    public void forward(int distance, double power) {
        resetEncoders();

        int target = (int) (distance * COUNTS_PER_INCH);
        leftFront_motor.setTargetPosition(-target);
        leftBack_motor.setTargetPosition(-target);
        rightFront_motor.setTargetPosition(-target);
        rightBack_motor.setTargetPosition(-target);

        leftFront_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Left Motor velocity is", leftFront_motor.getPower());
        telemetry.addData("Right Motor velocity is", rightFront_motor.getPower());
        telemetry.update();

        setMotorPower(power);
        slowDownAtEnd(power);
        stopMotor();
    }

    private void setMotorPower(double x) {
        leftFront_motor.setPower(x);
        leftBack_motor.setPower(x);
        rightFront_motor.setPower(x);
        rightBack_motor.setPower(x);
    }

    public void backward(int distance, double power) {
        resetEncoders();

        int target = (int) (distance * COUNTS_PER_INCH);
        leftFront_motor.setTargetPosition(target);
        leftBack_motor.setTargetPosition(target);
        rightFront_motor.setTargetPosition(target);
        rightBack_motor.setTargetPosition(target);

        leftFront_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setMotorPower(power);
        slowDownAtEnd(power);

        stopMotor();
    }

    public void turn_Left(double degrees, double power) {
        resetEncoders();

        double radians = Math.toRadians(degrees);
        double wheelDistance = radians * (ROBOT_WIDTH_INCHES / 2);

        int encoderCounts = (int) (wheelDistance * COUNTS_PER_INCH);

        // Set the target position for a left turn
        leftFront_motor.setTargetPosition(encoderCounts);
        leftBack_motor.setTargetPosition(encoderCounts);
        rightFront_motor.setTargetPosition(-encoderCounts);
        rightBack_motor.setTargetPosition(-encoderCounts);

        leftFront_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Power the motors to move towards the target position
        setMotorPower(power);

        slowDownAtEnd(power);

        stopMotor();
    }

    public void turn_Right(int degrees, double power) {
        resetEncoders();

        double radians = Math.toRadians(degrees);
        double wheelDistance = radians * (ROBOT_WIDTH_INCHES / 1.4);

        int encoderCounts = (int) (wheelDistance * COUNTS_PER_INCH);

        // Set the target position for a left turn
        leftFront_motor.setTargetPosition(-encoderCounts);
        leftBack_motor.setTargetPosition(-encoderCounts);
        rightFront_motor.setTargetPosition(encoderCounts);
        rightBack_motor.setTargetPosition(encoderCounts);

        leftFront_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Power the motors to move towards the target position
        setMotorPower(power);

        slowDownAtEnd(power);

        stopMotor();
    }

    public void strafe_Right(float distance, double power) {
        resetEncoders();

        int target = (int) (distance * COUNTS_PER_INCH);
        leftFront_motor.setTargetPosition(-target);
        leftBack_motor.setTargetPosition(target);
        rightFront_motor.setTargetPosition(target);
        rightBack_motor.setTargetPosition(-target);

        leftFront_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setMotorPower(power);
        slowDownAtEnd(power);

        stopMotor();

    }

    public void strafe_Left(float distance, double power) {
        resetEncoders();

        int target = (int) (distance * COUNTS_PER_INCH);
        leftFront_motor.setTargetPosition(target);
        leftBack_motor.setTargetPosition(-target);
        rightFront_motor.setTargetPosition(-target);
        rightBack_motor.setTargetPosition(target);

        leftFront_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setMotorPower(power);
        slowDownAtEnd(power);
        stopMotor();

    }

    public void stopMotor() {
        setMotorPower(0);
        resetEncoders();
    }

    private void resetEncoders() {
        leftFront_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}