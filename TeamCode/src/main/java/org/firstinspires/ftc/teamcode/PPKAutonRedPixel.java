package org.firstinspires.ftc.teamcode;

//import android.widget.Switch;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
import java.util.List;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.robotcore.external.JavaUtil;

@Autonomous(name="PPKAutonRedPixel", group = "Auto")
public class PPKAutonRedPixel extends LinearOpMode {
    private DcMotor leftFront_motor;

    private DcMotor leftBack_motor;
    private DcMotor rightFront_motor;

    private DcMotor rightBack_motor;
    private DcMotor intake_motor;
    private Servo pusher_servo;
    private Servo gate_servo;
    private Servo scooper_servo;

    private DcMotor arm_motor;

    private Servo wrist_servo;
    private Servo claw_servo;
    private Servo telescope_servo;

    public enum PropPostition {
        LEFT, CENTER, RIGHT, UNKNOWN

    }
    public enum ParkingLocation {
        IN, OUT

    }
    public ParkingLocation parkLocation;

    public PropPostition spikePosition;

    // Meccanum Drive Train Variables.
    // COUNTS_PER_MOTOR_REV should match the Encoder Resolution of the motor being used
    // for 312 RPM motor this value is 537
    // for 30 RPM motor this value is 5281
    // always check the gobilda motor specs
    // Wheel diameter should be accurate - changing from 3.75 to 3.78 as per gobilda
    private static final double COUNTS_PER_MOTOR_REV_DRIVE = 537;
    private static final double WHEEL_DIAMETER_INCHES = 3.78;
    private static final double GEAR_RATIO = 1.0;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV_DRIVE * GEAR_RATIO) / (WHEEL_DIAMETER_INCHES * Math.PI);
    private static final double ROBOT_WIDTH_INCHES = 18.9; // The distance between the wheels on opposite sides (diagonal)

    // arm motor calculations for degree changes
    private static final double COUNTS_PER_MOTOR_REV_ARM = 5281;
    private static final double ARM_DEGREES_PER_TIC = COUNTS_PER_MOTOR_REV_ARM / 360;
    private int arm_degree_position = 190;
    private int arm_tic_value = (int) Math.round(arm_degree_position * ARM_DEGREES_PER_TIC);


    // Set Motor power values.
    private double motorPowerValue = 1;
    private double intakePowerValue = 0.4;
    private double armPowerValue = 0.6;

    // Set SERVO values.

    private static final double WRIST_BACKDROP_DROP = 0.55;
    private static final double WRIST_BACKDROP_EXTEND_DROP = 0.75;
    private static final double WRIST_BACKDROP_RESET = 0.8;
    private static final double CLAW_OPEN = 0.2;
    private static final double CLAW_CLOSE = 0.5;

    private static final double PUSHER_RESET = 0.995;
    private static final double GATE_RESET = 0.0;
    private static final double TELESCOPE_RESET = 0.0;

    private static final double SCOOPER_MAX_POSITION = 220.0/270.0; // normalized for 270 degrees

    private boolean purplePixelDropped = false;
    private boolean firstContourDetected = false;


    // Set Camera related variables
    OpenCvCamera webcam;
    SamplePipeline pipeline;

    // Set AprilTag related variables
    //public AprilTagProcessor aprilTag;
    //public VisionPortal visionPortal;
    //public int AprilTagDetected = 0;
    //public int WantedAprilTag;
    //public boolean SearchAprilTAG = false;

    // Set Timer variables
    //private ElapsedTime runtime = new ElapsedTime(); //timer, used to remove sleeps
    private int sleepSecond = 1000;
    private int turnSleepSecond = 1000;

    @Override
    public void runOpMode() {
        telemetry.addLine("Please ensure to run the camera test before you run this part");
        telemetry.update();
        DriveInitialization();
        CameraInitialization();
        pipeline.startProcessing = false;
        parkLocation = ParkingLocation.IN;

        telemetry.addLine("All Initialization Completed");
        telemetry.update();

        //AprilTAGInitialization();
        waitForStart();
        telemetry.clear();

        pipeline.startProcessing = true;
        firstContourDetected = false;

        while (opModeIsActive() && !firstContourDetected) {
            // Identify on which pixel is the prop on.
            spikePosition = pipeline.getLastDetectedPosition();

            if (spikePosition != PropPostition.UNKNOWN) {
                firstContourDetected = true;
                telemetry.addLine("Prop Detected");
                telemetry.addData("Position", spikePosition);
                telemetry.update();
                webcam.stopStreaming();
            }
            // Telemetry (optional)
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.update();

        }

        telemetry.addLine("Ready to drop purple");
        telemetry.update();
        DropPurplePixel(spikePosition);
        telemetry.addLine("Purple Pixel Dropped");
        telemetry.addLine("Lets go backstage");
        telemetry.update();
        sleep(sleepSecond);

        NavigateToBackstage(spikePosition);
        telemetry.addLine("Yellow Pixel Ready to Drop");
        telemetry.update();
        sleep(sleepSecond);
        switch(spikePosition) {
            case CENTER:
                yellow_pixel_drop(); // drop the yellow pixel
                telemetry.addLine("Yellow Pixel Dropped");
                telemetry.update();
                sleep(sleepSecond);

                //Set for driver mode
                scooper_servo.setPosition(SCOOPER_MAX_POSITION);
                arm_motor.setPower(armPowerValue);
                arm_motor.setTargetPosition(0);
                arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(sleepSecond);
                ParkBackstage(spikePosition, parkLocation);
                backward(5, motorPowerValue);
                break;


            case RIGHT:
                break;


            case LEFT:
                break;


        }

//        yellow_pixel_drop(); // drop the yellow pixel
//        telemetry.addLine("Yellow Pixel Dropped");
//        telemetry.update();
//        sleep(sleepSecond);
//
//        //Set for driver mode
//        scooper_servo.setPosition(SCOOPER_MAX_POSITION);
//        arm_motor.setPower(armPowerValue);
//        arm_motor.setTargetPosition(0);
//        arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        sleep(sleepSecond);
//        ParkBackstage(spikePosition, parkLocation);
//        backward(5, motorPowerValue);
        webcam.stopStreaming(); // Stop streaming when the op mode is no longer active
        telemetry.update();
        sleep(sleepSecond);

    }

    // All Initialization Code
    public void DriveInitialization() {
        leftFront_motor = hardwareMap.get(DcMotor.class, "left_front");
        leftBack_motor = hardwareMap.get(DcMotor.class, "left_back");
        rightFront_motor = hardwareMap.get(DcMotor.class, "right_front");
        rightBack_motor = hardwareMap.get(DcMotor.class, "right_back");
        arm_motor = hardwareMap.get(DcMotor.class, "arm_motor");
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

        // New Servo & Motors
        arm_motor = hardwareMap.get(DcMotor.class, "arm_motor");
        arm_motor.setDirection(DcMotor.Direction.REVERSE);
        arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // set its current position to 0 - so this can be used for all times.

        arm_motor.setTargetPosition(0);
        arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        claw_servo = hardwareMap.get(Servo.class, "claw_servo");
        claw_servo.setPosition(CLAW_CLOSE);

        telescope_servo = hardwareMap.get(Servo.class, "telescoping_servo");
        wrist_servo = hardwareMap.get(Servo.class, "wrist_servo");
        wrist_servo.setDirection(Servo.Direction.FORWARD);

        //wrist_servo.setPosition(WRIST_BACKDROP_RESET);

        telemetry.addLine("Motor & Servo init. completed");
    }

    private void CameraInitialization(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new SamplePipeline();
        pipeline.startProcessing = false;
        webcam.setPipeline(pipeline); // new to ensure pipeline not looking for team prop on initialization
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
        telemetry.addLine("Camera Initialization.. completed");
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

            Scalar lowerRed = new Scalar(0, 50, 50); //middle unit was 150
            Scalar upperRed = new Scalar(10, 255, 255); //first unit was 20
            //otherwise test (160 and 180 for the first unit)
            Mat PropMask = new Mat();
            Core.inRange(hsvImage, lowerRed, upperRed, PropMask);

            List<org.opencv.core.MatOfPoint> contours = new ArrayList<>();
            Mat hierachy = new Mat();
            Imgproc.findContours(PropMask, contours, hierachy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            for (org.opencv.core.MatOfPoint contour : contours) {
                if (Imgproc.contourArea(contour) > 10500) {
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
            PropMask.release();
            hierachy.release();

            return input;
        }

        public PropPostition getLastDetectedPosition() {
            return lastDetectedPosition;
        }
    }

    private void DropPurplePixel(PropPostition spikePosition) {
        //till reverse intake
        PropPostition spikePosition1 = spikePosition;
        int arm_tic_value_2;
        switch (spikePosition1) {
            case LEFT:
                //left logic
//                telemetry.addLine("RIGHT, going for LEFT purple pixel drop");
//                telemetry.update();
//                forward(29, motorPowerValue);
//                turn_Left(95, motorPowerValue);
//                sleep(turnSleepSecond);
//                forward(6, motorPowerValue);
//                backward(3.5F,motorPowerValue);
//                intake_motor.setPower(intakePowerValue);
//                sleep(sleepSecond);
//                intake_motor.setPower(0);
//                backward(2,motorPowerValue);
                break;

            case CENTER:
                //center logic
                telemetry.addLine("CENTER, going for CENTER purple pixel drop");
                telemetry.update();
                forward(30, motorPowerValue);
                //backward(2, motorPowerValue);
                intake_motor.setPower(intakePowerValue);
                sleep(sleepSecond);
                intake_motor.setPower(0);
                backward(4.8F, motorPowerValue);
                break;

            case RIGHT:
                //right logic
//                telemetry.addLine("LEFT, going for LEFT purple pixel drop");
//                telemetry.update();
//                forward(30, motorPowerValue);
//                turn_Right(90, motorPowerValue);
//                sleep(turnSleepSecond);
//                forward(7.5F, motorPowerValue);
//                backward(3.5F,motorPowerValue);
//                intake_motor.setPower(intakePowerValue);
//                arm_motor.setPower(armPowerValue);
//                arm_motor.setTargetPosition((int) Math.round(20 * ARM_DEGREES_PER_TIC));
//                arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                sleep(750);
//                intake_motor.setPower(0);
                break;

            case UNKNOWN:
                //unknown logic
                telemetry.addLine("UNKNOWN, so do nothing");
//                telemetry.update();
//                forward(30, motorPowerValue);
//                intake_motor.setPower(intakePowerValue);
//                sleep(sleepSecond);
//                intake_motor.setPower(0);
//                backward(4.8F, motorPowerValue);
                break;
        }
    }

    private void NavigateToBackstage(PropPostition spikePosition) {
        PropPostition spikePosition1 = spikePosition;
        switch (spikePosition) {
            case LEFT:
                //left logic
//                telemetry.addLine("RIGHT, Navigate BackStage");
//                telemetry.update();
                break;
//                arm_motor.setPower(armPowerValue);
//                arm_motor.setTargetPosition((int) Math.round(20 * ARM_DEGREES_PER_TIC));
//                arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                backward(3,motorPowerValue);
//                strafe_Right(26,motorPowerValue);
//                scooper_servo.setPosition(SCOOPER_MAX_POSITION);
//                sleep(turnSleepSecond);
//                arm_motor.setPower(armPowerValue);
//                arm_motor.setTargetPosition(0);
//                arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                wrist_servo.setPosition(WRIST_BACKDROP_DROP);
//                backward(80, motorPowerValue);
//                strafe_Left(16,motorPowerValue);
//
//                telemetry.addLine("Ready for arm");
//                telemetry.update();
//                arm_backstage(armPowerValue, arm_tic_value);
//                break;

            case CENTER:
                //center logic
                telemetry.addLine("CENTER, Navigate BackStage");
                telemetry.update();

                strafe_Left(15,motorPowerValue);
                arm_motor.setPower(armPowerValue);
                arm_motor.setTargetPosition((int) Math.round(20 * ARM_DEGREES_PER_TIC));
                arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                forward(26,motorPowerValue);
                turn_Left(95,motorPowerValue);
                scooper_servo.setPosition(SCOOPER_MAX_POSITION);
                sleep(turnSleepSecond);
                arm_motor.setPower(armPowerValue);
                arm_motor.setTargetPosition(0);
                arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist_servo.setPosition(WRIST_BACKDROP_DROP);
                backward(95, motorPowerValue);
                strafe_Left(27,motorPowerValue);
                telemetry.addLine("Ready for arm");
                telemetry.update();
                arm_backstage(armPowerValue, arm_tic_value);
                break;

            case RIGHT:
                //right logic
//                telemetry.addLine("Right, Navigate BackStage");
//                telemetry.update();
//                backward(3,motorPowerValue);
//                scooper_servo.setPosition(SCOOPER_MAX_POSITION);
//                strafe_Left(26,motorPowerValue);
//                //sleep(300);
//                arm_motor.setPower(armPowerValue);
//                arm_motor.setTargetPosition(0);
//                arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                wrist_servo.setPosition(WRIST_BACKDROP_DROP);
//                //test
//                forward(40,motorPowerValue);
//                turn_Right(2,motorPowerValue);
//                forward(44,motorPowerValue);
//                strafe_Right(43F,motorPowerValue);
//                turn_Right(180,motorPowerValue);
//                sleep(turnSleepSecond+750);

//                turn_Left(183,motorPowerValue);
//                sleep(turnSleepSecond+750);
//                backward(82, motorPowerValue);
//                strafe_Right(38.5F,motorPowerValue);
//                telemetry.addLine("Ready for arm");
//                telemetry.update();
//                arm_backstage(armPowerValue, arm_tic_value);
                break;

            case UNKNOWN:
                //Unknown logic
//                telemetry.addLine("UNKNOWN, Navigate BackStage");
//                telemetry.update();
//                turn_Right(90, motorPowerValue);
//                sleep(turnSleepSecond);
//                backward(34, motorPowerValue);  // this might need to change since the motor specs has been corrected
//                telemetry.addLine("Ready for arm");
//                telemetry.update();
//                arm_backstage(armPowerValue, arm_tic_value);
                break;
        }
    }

    public void arm_backstage(double armPowerValue, int arm_tic_value){
        //double arm_tic_value = arm_tic_value;
        int arm_tic_value_2 = arm_tic_value;

        telemetry.addLine("working on arm");
        arm_motor.setPower(armPowerValue);

        arm_motor.setTargetPosition(arm_tic_value_2);
        arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //position the wrist for drop
        wrist_servo.setPosition(WRIST_BACKDROP_DROP); //0.55 value

        telemetry.addLine("Arm Ready to drop");
        telemetry.addLine("Wrist done");
        telemetry.update();

        sleep(sleepSecond);


    }
    public void yellow_pixel_drop(){
        //position the wrist for drop
        claw_servo.setPosition(CLAW_OPEN);

        telemetry.addLine("Claw opened, Yellow dropped");
        telemetry.update();

    }

    private void ParkBackstage(PropPostition spikePosition, ParkingLocation parkLocation) {
        PropPostition spikePosition1 = spikePosition;
        ParkingLocation parkLocation1 = parkLocation;

        switch (spikePosition1) {
            case LEFT:
                //left logic
                telemetry.addLine("LEFT, BackStage");
                telemetry.update();
                switch (parkLocation1){
                    case IN:
                        telemetry.addLine("Park IN");
                        telemetry.update();
                        strafe_Right(35, motorPowerValue); //move away from the backstage

                        break;
                    case OUT:
                        telemetry.addLine("Park OUT");
                        telemetry.update();
                        strafe_Left(10, motorPowerValue); //move away from the backstage
                        break;

                }
                break;
            case CENTER:
                //left logic
                telemetry.addLine("CENTER, BackStage");
                telemetry.update();
                switch (parkLocation1){
                    case IN:
                        telemetry.addLine("Park IN");
                        telemetry.update();
                        strafe_Right(24, motorPowerValue); //move away from the backstage

                        break;
                    case OUT:
                        telemetry.addLine("Park OUT");
                        telemetry.update();
                        strafe_Left(20, motorPowerValue); //move away from the backstage
                        break;

                }
                break;
            case RIGHT:
                //left logic
                telemetry.addLine("RIGHT, BackStage");
                telemetry.update();
                switch (parkLocation1){
                    case IN:
                        telemetry.addLine("Park IN");
                        telemetry.update();
                        strafe_Right(15, motorPowerValue); //move away from the backstage
                        break;
                    case OUT:
                        telemetry.addLine("Park OUT");
                        telemetry.update();
                        strafe_Left(30, motorPowerValue); //move away from the backstage
                        break;

                }
                break;


        }
    }

    public void readyfordrive(){
        //get ready for drive
        claw_servo.setPosition(CLAW_CLOSE);
        arm_motor.setPower(armPowerValue);
        arm_motor.setTargetPosition(0);
        arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wrist_servo.setPosition(WRIST_BACKDROP_RESET);
        pusher_servo.setPosition(PUSHER_RESET);
        scooper_servo.setPosition(SCOOPER_MAX_POSITION);
        gate_servo.setPosition(GATE_RESET);
        telescope_servo.setPosition(TELESCOPE_RESET);

        telemetry.addLine("Reset Ready to drive");
        telemetry.update();

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

    public void forward(float distance, double power) {
        resetDriveEncoders();

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

    public void backward(float distance, double power) {
        resetDriveEncoders();

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
        resetDriveEncoders();

        double radians = Math.toRadians(degrees);
        double wheelDistance = radians * (ROBOT_WIDTH_INCHES / 1.4);

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
        resetDriveEncoders();

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
        resetDriveEncoders();

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
        resetDriveEncoders();

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
        resetDriveEncoders();
    }

    private void resetDriveEncoders() {
        leftFront_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


//    public int DetectAprilTag(boolean SearchAprilTAG) {
//        if (!SearchAprilTAG) {
//            return 0;
//        }
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null) {
//                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//
//                return (detection.id);
//
//            } else {
//                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//            }
//        }
//        return 0;
//    }
//    private void DropYellowPixel(PropPostition spikePosition) {
//        AprilTagDetected = 0;
//        telemetry.addData("SearchAprilTAG", SearchAprilTAG);
//        SearchAprilTAG = false;
//
//        if (runtime.milliseconds() > 5000) {
//            SearchAprilTAG = true;
//        }
//
//        AprilTagDetected = DetectAprilTag(SearchAprilTAG);
//        telemetry.addData("SearchAprilTAG", SearchAprilTAG);
//        telemetry.addData("AprilTagDetected", AprilTagDetected);
//
//        PropPostition spikePosition1 = spikePosition;
//        switch (spikePosition) {
//            case LEFT:
//                telemetry.addData("AprilTagDetected", AprilTagDetected);
//                telemetry.addLine("Detected 4");
//                if (AprilTagDetected == WantedAprilTag) {
//                    telemetry.addLine("Drop Pixel on Backdrop");
//                    visionPortal.close();
//                }
//                //code for arm placing on backdrop here
//                strafe_Left(12, motorPowerValue);
//                break;
//
//            case CENTER:
//                telemetry.addData("AprilTagDetected", AprilTagDetected);
//                telemetry.addLine("Detected 5");
//                if (AprilTagDetected == WantedAprilTag) {
//                    telemetry.addLine("Drop Pixel on Backdrop");
//                    visionPortal.close();
//                }
//                //code for arm placing on backdrop here
//                strafe_Left(18, motorPowerValue);
//                break;
//
//            case RIGHT:
//                telemetry.addData("AprilTagDetected", AprilTagDetected);
//                telemetry.addLine("Detected 6");
//                if (AprilTagDetected == WantedAprilTag) {
//                    telemetry.addLine("Drop Pixel on Backdrop");
//                    visionPortal.close();
//                }
//                //code for arm placing on backdrop here
//                strafe_Left(24, motorPowerValue);
//                break;
//
//            case UNKNOWN:
//                telemetry.addData("AprilTagDetected", AprilTagDetected);
//                telemetry.addLine("Unable to detect April Tag");
//                break;
//        }
//        telemetry.update();
//    }
//
//    public void AprilTAGInitialization() {
//
//        // Create the AprilTag processor.
//        aprilTag = new AprilTagProcessor.Builder().build();
//
//        // Create the vision portal by using a builder.
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//
//        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        builder.addProcessor(aprilTag);
//
//        // Build the Vision Portal, using the above settings.
//        visionPortal = builder.build();
//
//    }

}
