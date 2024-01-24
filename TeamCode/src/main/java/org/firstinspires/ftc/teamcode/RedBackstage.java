package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="RedBackstage", group = "Auto")
public class RedBackstage extends LinearOpMode {

    public enum PropPostition {
        LEFT, CENTER, RIGHT, UNKNOWN
    }

    public enum ParkingLocation {
        IN, OUT

    }
    public ParkingLocation parkLocation;

    public PropPostition spikePosition;
    private boolean purplePixelDropped = false;
    private boolean firstContourDetected = false;


    // Set Camera related variables
    OpenCvCamera webcam;
    SamplePipeline pipeline;

    //Set Gyro and Drive related variables
    private DcMotor     frontLeft   = null;
    private DcMotor     frontRight  = null;
    private DcMotor     backLeft   = null;
    private DcMotor     backRight  = null;
    private DcMotor     intake     = null;
    private Servo       wristServo = null;
    private CRServo leftWristServo;
    private CRServo rightWristServo;
    private DcMotor rightLinearSlide_motor;
    private DcMotor leftLinearSlide_motor;
    private IMU         imu        = null;      // Control/Expansion Hub IMU
    private double      headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double strafeSpeed    = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
    private int     leftTarget    = 0;
    private int     rightTarget   = 0;

    private ElapsedTime timer;
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.78;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    private static final double ROBOT_DIAMETER_INCHES = 27; // Adjust to your robot's diameter
    // The distance between the wheels on opposite sides (diagonal)
    static final double     DRIVE_SPEED = 0.995;     // Max driving speed for better distance accuracy.
    static final double  TURN_IMU_SPEED =0.3;
    static final double     TURN_SPEED = 0.4;
    static final double     STRAFE_SPEED  = 0.4;
    static final double     HEADING_THRESHOLD  = 1.0 ;
    static final double     LINEAR_SLIDE_POWER = 0.8;
    static final double     LINEAR_SLIDE_TICKS = 28 * COUNTS_PER_INCH; //distance * encoder counts_per_inch


    static final double     P_TURN_GAIN  = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN  = 0.03;     // Larger is more responsive, but also less stable

    private ElapsedTime runtime = new ElapsedTime();
    ElapsedTime linearSlideTimer;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Please ensure to run the camera test before you run this part");
        telemetry.update();
        //CameraInitialization();
        DriveInitialization();
        //CameraInitialization();
        //pipeline.startProcessing = false;

        telemetry.addLine("All Initialization Completed");
        telemetry.update();

        while (opModeInInit()) {
            //for gyro sensor
            targetHeading = getHeading();
            telemetry.addData(">", "Robot Heading = %4.0f", targetHeading);
            telemetry.update();
            wristServo.setPosition(0);
        }

        waitForStart();
        spikePosition = PropPostition.CENTER;
        runAuton(spikePosition);
        sleep(2000);
    }


//        telemetry.clear();
//
//        pipeline.startProcessing = true;
//        firstContourDetected = false;
//
//        long startTime = System.currentTimeMillis();
//
//        while (opModeIsActive() && !firstContourDetected) {
//            // Identify on which pixel is the prop on.
//            spikePosition = pipeline.getLastDetectedPosition();
//
//            if (spikePosition != PropPostition.UNKNOWN) {
//                firstContourDetected = true;
//                telemetry.addLine("Prop Detected");
//                telemetry.addData("Position", spikePosition);
//                telemetry.update();
//                //sleep(1000);
//                webcam.stopStreaming();
//            }
//            // Check the elapsed time
//            long elapsedTime = System.currentTimeMillis() - startTime;
//            if (elapsedTime > 7000) { // 7000 milliseconds equals 7 seconds
//                spikePosition = PropPostition.CENTER;
//                webcam.stopStreaming();
//                telemetry.addLine("Prop Detected UNKOWN - move to backup plan");
//                telemetry.addData("Position", spikePosition);
//                telemetry.update();
//                break; // Break out of the loop
//            }
//            // Telemetry (optional)
//            telemetry.addData("Frame Count", webcam.getFrameCount());
//            telemetry.update();
//        }
//
//        runAuton(spikePosition);
//
//        //spikePosition = CENTER;
//        //purplePixelDrop(spikePosition);
//        //NavigateToBackstage(spikePosition);
//        //dropYellowPixel();
//        //ParkBackstage(spikePosition, ParkingLocation.OUT);
//        telemetry.addData("Path", "Complete");
//        telemetry.update();
//        sleep(1000);  // Pause to display last telemetry message.
//        stopRobot();
//    }

    public void runAuton(PropPostition  spikePosition){
        PropPostition final_spikePosition = spikePosition;

        switch (final_spikePosition){
            case UNKNOWN:
                telemetry.addLine("UNKNOWN, so trial and error on center logic");
            case CENTER:
                telemetry.addLine("CENTER, going for CENTER purple pixel drop");
                telemetry.update();
                drive_forward(27,DRIVE_SPEED);
                intake.setPower(0.4);
                holdHeading(TURN_IMU_SPEED,0,2.1);
                drive_backward(1.25, DRIVE_SPEED);
                intake.setPower(0);
                drive_backward(0.5, DRIVE_SPEED);
                turnLeft(94, TURN_IMU_SPEED);
                drive_backward(40.99, DRIVE_SPEED);
                dropYellowPixel();
                break;
            case LEFT:
                drive_forward(28, DRIVE_SPEED);
                turnLeft(91, TURN_IMU_SPEED);
                drive_forward(1.5, DRIVE_SPEED);
                intake.setPower(0.4);
                holdHeading(TURN_SPEED, 0, 1.55);
                drive_backward(3, DRIVE_SPEED);
                intake.setPower(0);
                drive_backward(31, DRIVE_SPEED);
                strafe_Right(6, STRAFE_SPEED);
                dropYellowPixel();
                break;
            case RIGHT:
                drive_forward(2, DRIVE_SPEED);
                strafe_Right(13.25F, STRAFE_SPEED);
                drive_forward(22, DRIVE_SPEED);
                intake.setPower(0.4);
                holdHeading(TURN_SPEED, 0, 1.25);
                drive_backward(3, DRIVE_SPEED);
                intake.setPower(0);
                turnLeft(91, TURN_IMU_SPEED);
                drive_backward(31, DRIVE_SPEED);
                dropYellowPixel();
                break;

        }
    }

    public void purplePixelDrop(PropPostition spikePosition){
        PropPostition spikePosition1 = spikePosition;
        switch (spikePosition1) {
            case RIGHT:
                //left logic
                telemetry.addLine("RIGHT, going for RIGHT purple pixel drop");
                telemetry.update();
                driveStraight(DRIVE_SPEED, 28, 0);
                turnToHeading(TURN_SPEED, -91);
                holdHeading(TURN_SPEED, -91, 0.5);
                driveStraight(DRIVE_SPEED, 1.5, 0);
                intake.setPower(0.4);
                sleep(1550);
                driveStraight(DRIVE_SPEED, -3, 0);
                intake.setPower(0);
                //driveStraight(DRIVE_SPEED, -8, 0);
                break;

            case CENTER:
                //center logic
                telemetry.addLine("CENTER, going for CENTER purple pixel drop");
                telemetry.update();
                driveStraight(DRIVE_SPEED, 28, 0);
                intake.setPower(0.375);
                sleep(1200);
                driveStraight(DRIVE_SPEED, -5, 0);
                intake.setPower(0);
                //driveStraight(DRIVE_SPEED, -8, 0);
                break;

            case LEFT:
                //right logic
                telemetry.addLine("LEFT, going for LEFT purple pixel drop");
                telemetry.update();
                driveStraight(DRIVE_SPEED, 2, 0);
                strafe(STRAFE_SPEED, -13.25F, 0);
//                turnToHeading(TURN_SPEED, 0);
//                holdHeading(TURN_SPEED, 0, 0.5);
                driveStraight(DRIVE_SPEED, 22, 0);
                intake.setPower(0.4);
                sleep(1250);
                driveStraight(DRIVE_SPEED, -3, 0);
                intake.setPower(0);
                //driveStraight(DRIVE_SPEED, -4.8, 0);
                break;

            case UNKNOWN:
                //unknown logic
                telemetry.addLine("UNKNOWN, so trial and error on center logic");

                break;

        }


    }
    private void NavigateToBackstage(PropPostition spikePosition) {
        PropPostition spikePosition1 = spikePosition;
        switch (spikePosition1) {
            case LEFT:
                //left logic
                telemetry.addLine("LEFT, Navigate BackStage");
                telemetry.update();
                driveStraight(DRIVE_SPEED, -31, 0);
                strafe(STRAFE_SPEED, 6, 0);
                break;

            case CENTER:
                //center logic
                telemetry.addLine("CENTER, Navigate BackStage");
                telemetry.update();
                strafe(STRAFE_SPEED,-15,0);
                driveStraight(DRIVE_SPEED, 23, 0);
                strafe(STRAFE_SPEED,96,0);
                driveStraight(DRIVE_SPEED,-20,0);
                turnToHeading(TURN_SPEED,-10);
                holdHeading(TURN_SPEED,-10,0.5);
//                strafe(STRAFE_SPEED,32,0);
                //strafe(STRAFE_SPEED,-15,0);

//                turnToHeading(TURN_SPEED, 90);
//                holdHeading(TURN_SPEED, 90, 0.5);
                break;

            case RIGHT:
                //right logic
                telemetry.addLine("RIGHT, Navigate BackStage");
                telemetry.update();
                turnToHeading(TURN_SPEED, 92);
//                strafe(STRAFE_SPEED, 2, 0);
                driveStraight(DRIVE_SPEED, -31, 0);
                break;

            case UNKNOWN:
                //Unknown logic
                telemetry.addLine("UNKNOWN, Navigate BackStage");

                break;
        }
    }

    public void dropYellowPixel(){
        linearSlideTimer.reset();
        rightLinearSlide_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLinearSlide_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLinearSlide_motor.setTargetPosition((int) LINEAR_SLIDE_TICKS);
        leftLinearSlide_motor.setTargetPosition((int) LINEAR_SLIDE_TICKS);
        rightLinearSlide_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLinearSlide_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLinearSlide_motor.setPower(LINEAR_SLIDE_POWER);
        leftLinearSlide_motor.setPower(LINEAR_SLIDE_POWER);

        while(leftLinearSlide_motor.isBusy() && rightLinearSlide_motor.isBusy() && linearSlideTimer.seconds() < 5) {
            telemetry.addLine("waiting for linear slides to reach target position");
            telemetry.update();
        }

        rightLinearSlide_motor.setPower(0);
        leftLinearSlide_motor.setPower(0);

        wristServo.setPosition(1);
        leftWristServo.setPower(-0.995);
        sleep(2000);
        leftWristServo.setPower(0);
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

                        break;
                    case OUT:
                        telemetry.addLine("Park OUT");
                        telemetry.update();
                        driveStraight(DRIVE_SPEED, 3, 0);
                        strafe(STRAFE_SPEED, -30, 0);
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


                        break;
                    case OUT:
                        telemetry.addLine("Park OUT");
                        telemetry.update();
                        driveStraight(DRIVE_SPEED, 3, 0);
                        strafe(STRAFE_SPEED, -11, 0);//move away from the backstage
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
                        break;
                    case OUT:
                        telemetry.addLine("Park OUT");
                        telemetry.update();
                        driveStraight(DRIVE_SPEED, 3, 0);
                        strafe(STRAFE_SPEED, -20, 0);
                        break;

                }
                break;


        }
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

        private static final double LEFT_REGION = 0.33; //0.33
        private static final double RIGHT_REGION = 0.66; //0.66
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

            Scalar lowerRed = new Scalar(0, 50, 50); //Dec 2 values: (middle unit was 150)
            Scalar upperRed = new Scalar(10, 255, 255); //Dec 2 values: (first unit was 20)
            //otherwise test (160 and 180 for the first unit)
            Mat PropMask = new Mat();
            Core.inRange(hsvImage, lowerRed, upperRed, PropMask);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierachy = new Mat();
            Imgproc.findContours(PropMask, contours, hierachy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            //initial contour "greater than value" was 10500
            for (MatOfPoint contour : contours) {
                if (Imgproc.contourArea(contour) > 10000) {
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
    public void DriveInitialization() {
        // Initialize the drive system variables.
        frontLeft  = hardwareMap.get(DcMotor.class, "left_front");
        backLeft = hardwareMap.get(DcMotor.class, "left_back");
        frontRight = hardwareMap.get(DcMotor.class, "right_front");
        backRight = hardwareMap.get(DcMotor.class, "right_back");
        wristServo = hardwareMap.get(Servo.class, "wristservo");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftWristServo = hardwareMap.get(CRServo.class, "leftwristservo");
        rightLinearSlide_motor = hardwareMap.get(DcMotor.class, "rightlinearslide");
        leftLinearSlide_motor = hardwareMap.get(DcMotor.class, "leftlinearslide");
        rightLinearSlide_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLinearSlide_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLinearSlide_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLinearSlide_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideTimer = new ElapsedTime();
        rightLinearSlide_motor.setDirection(DcMotor.Direction.FORWARD);
        leftLinearSlide_motor.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(DcMotor.Direction.FORWARD);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Wait for the game to start (Display Gyro value while waiting)
        // Set the encoders for closed loop speed control, and reset the heading.
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu.resetYaw();

        telemetry.addLine("Drive Initialization is complete");
    }

    public void driveStraight(double maxDriveSpeed, double distance, double heading) {
        setRunUsingEncoder();
        resetDriveEncoders();

        if (opModeIsActive()) {
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            setDriveTargetPosition(moveCounts, moveCounts, moveCounts, moveCounts);
            //sleep(1000);
            setRunToPosition();
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0,0);

            while (opModeIsActive() && (frontLeft.isBusy() || frontRight.isBusy() || backRight.isBusy() || backLeft.isBusy())) {
                //
//                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
//
//                // if driving in reverse, the motor correction also needs to be reversed
//                if (distance < 0)
//                    turnSpeed *= -1.0;
//
//                // Apply the turning correction to the current driving speed.
//                moveRobot(maxDriveSpeed,0, turnSpeed);
//
//                // Display drive status for the driver.
//                sendTelemetry(true);

            }

        }
        setPowerDriveTrain(0);
        setRunUsingEncoder();

    }
    public void strafe(double maxDriveSpeed, double distance, double heading) {
        resetDriveEncoders();
        if (opModeIsActive()) {
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            //void setDriveTargetPosition(int frontLeftTarget, int frontRightTarget, int backLeftTarget, int backRightTarget)
            setDriveTargetPosition(moveCounts, -moveCounts, -moveCounts, moveCounts);
            //sleep(1000);
            setRunToPosition();
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(0, maxDriveSpeed,0);

            while (opModeIsActive() && (frontLeft.isBusy() || frontRight.isBusy() || backRight.isBusy() || backLeft.isBusy())) {
                //
//                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
//
//                // if driving in reverse, the motor correction also needs to be reversed
//                if (distance < 0)
//                    turnSpeed *= -1.0;
//
//                // Apply the turning correction to the current driving speed.
//                moveRobot(0,maxDriveSpeed, turnSpeed);
//
//                // Display drive status for the driver.
//                sendTelemetry(true);

            }

        }
        setPowerDriveTrain(0);
        setRunUsingEncoder();

    }
    public void turnToHeading(double maxTurnSpeed, double heading) {

        telemetry.addData("In Turn Function", "In Progress");
        telemetry.update();

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);
        //sleep(1000);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
            telemetry.addData("In Turn Function", "Loop In Progress");

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            telemetry.addData("turnSpeed: ", turnSpeed);
            telemetry.update();

            // Pivot in place by applying the turning correction
            moveRobot(0,0 ,turnSpeed);

            // Display drive status
            // for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        setPowerDriveTrain(0);
        setRunUsingEncoder();

    }
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, 0,turnSpeed);

            // Display drive status for the driver.
            //sendTelemetry(false);
        }

        // Stop all motion;
        setPowerDriveTrain(0);
        setRunUsingEncoder();

    }
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        telemetry.addData(">", "targetHeading in getSteeringCorrection = %4.0f", targetHeading);
        telemetry.addData(">", "headingError in getSteeringCorrection = %4.0f", headingError);
        telemetry.update();
        sleep(500);

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    // **********  LOW Level driving functions.  ********************
    // Use a Proportional Controller to determine how much steering correction is required.
    // @param desiredHeading        The desired absolute heading (relative to last heading reset)
    // @param proportionalGain      Gain factor applied to heading error to obtain turning power.
    // @return                      Turning power needed to get to required heading.
    // Take separate drive (fwd/rev) and turn (right/left) requests,
    // combines them, and applies the appropriate speed commands to the left and right wheel motors.

    public void moveRobot(double drive, double strafe, double turn) {

//        driveSpeed = drive;     // forward/backward
//        turnSpeed = turn;       // rotation
//        strafeSpeed = strafe;   // left/right

        // Calculate speed for each wheel
        double frontLeftSpeed = drive - turn + strafe;
        double backLeftSpeed = drive - turn - strafe;
        double frontRightSpeed = drive + turn -strafe ;
        double backRightSpeed = drive + turn + strafe;

        // Scale speeds down if any one exceeds +/- 1.0
        double max = Math.max(Math.max(Math.abs(frontLeftSpeed), Math.abs(frontRightSpeed)),
                Math.max(Math.abs(backLeftSpeed), Math.abs(backRightSpeed)));

        if (max > 1.0) {
            frontLeftSpeed /= max;
            frontRightSpeed /= max;
            backLeftSpeed /= max;
            backRightSpeed /= max;
        }

        // Set power for each motor
        frontLeft.setPower(frontLeftSpeed);
        frontRight.setPower(frontRightSpeed);
        backLeft.setPower(backLeftSpeed);
        backRight.setPower(backRightSpeed);
    }
    public void setRunToPosition() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setRunUsingEncoder() {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setPowerDriveTrain(double speed) {
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);
    }
    public void stopRobot() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    public void resetDriveEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setDriveTargetPosition(int frontLeftTarget, int frontRightTarget, int backLeftTarget, int backRightTarget) {

        int frontLeftTarget_final = frontLeft.getCurrentPosition() + frontLeftTarget;
        int frontRightTarget_final = frontRight.getCurrentPosition() + frontRightTarget;
        int backLeftTarget_final = backLeft.getCurrentPosition() + backLeftTarget;
        int backRightTarget_final = backRight.getCurrentPosition() + backRightTarget;
        frontLeft.setTargetPosition(frontLeftTarget_final);
        backLeft.setTargetPosition(backLeftTarget_final);
        frontRight.setTargetPosition(frontRightTarget_final);
        backRight.setTargetPosition(backRightTarget_final);
        telemetry.addData("Front Left Target Position: ", frontLeftTarget_final);
        telemetry.addData("Front Right Target Position: ", frontRightTarget_final);
        telemetry.addData("Back Left Target Position: ", backLeftTarget_final);
        telemetry.addData("Back Right Target Position: ", backRightTarget_final);
        telemetry.update();
    }

    //Display the various control parameters while driving
    //@param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.

    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftTarget,  rightTarget);
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      frontLeft.getCurrentPosition(),
                    frontRight.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    //read the Robot heading directly from the IMU (in degrees)
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        telemetry.addData("GetHeading :", "%5.2f", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.update();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public void drive_forward(double distance, double maxDriveSpeed) {
        resetDriveEncoders();
        int moveCounts = (int) (distance * COUNTS_PER_INCH);
        setDriveTargetPosition(moveCounts, moveCounts, moveCounts, moveCounts);
        setRunToPosition();
        maxDriveSpeed = Math.abs(maxDriveSpeed);
        setPowerDriveTrain(maxDriveSpeed);
        slowDownAtEnd(maxDriveSpeed);
        setPowerDriveTrain(0);
    }

    public void slowDownAtEnd(double power) {
        while (opModeIsActive() && (frontLeft.isBusy() || frontRight.isBusy())) { //Accurate measures
            double remainingLeftDistance = Math.abs(frontLeft.getTargetPosition() - frontLeft.getCurrentPosition());
            double remainingRightDistance = Math.abs(frontRight.getTargetPosition() - frontRight.getCurrentPosition());
            // Adjust this threshold as needed.
            double rampDownThreshold = 200;  // For example, start ramping down when 300 encoder counts away from target.

            if ((remainingLeftDistance < rampDownThreshold) || (remainingRightDistance < rampDownThreshold)){
                double remainingDistance = Math.min(remainingLeftDistance, remainingRightDistance);
                double rampedSpeed = 0.5 * (power * (remainingDistance / rampDownThreshold));
                rampedSpeed = Math.max(rampedSpeed, 0.1); // Ensure you have a minimum speed to prevent the robot from stalling.
                rampedSpeed = Range.clip(rampedSpeed, 0, 1);
                setPowerDriveTrain(rampedSpeed);
            }
        }
    }

    public void drive_backward(double distance, double maxDriveSpeed) {
        resetDriveEncoders();
        int moveCounts = (int) (-distance * COUNTS_PER_INCH); // Notice the negative sign
        setDriveTargetPosition(moveCounts, moveCounts, moveCounts, moveCounts);
        setRunToPosition();
        maxDriveSpeed = Math.abs(maxDriveSpeed);
        setPowerDriveTrain(maxDriveSpeed);
        slowDownAtEnd(maxDriveSpeed);
        setPowerDriveTrain(0);
    }

    public void strafe_Right(double distance, double power) {
        resetDriveEncoders();

        int target = (int) (distance * COUNTS_PER_INCH);
        setDriveTargetPosition(target,-target,-target,target);
        setRunToPosition();
        setPowerDriveTrain(power);
        slowDownAtEnd(power);
        setPowerDriveTrain(0);
    }
    public void strafe_Left(double distance, double power) {
        resetDriveEncoders();

        int target = (int) (distance * COUNTS_PER_INCH);
        setDriveTargetPosition(-target,target,target,-target);
        setRunToPosition();
        setPowerDriveTrain(power);
        slowDownAtEnd(power);
        setPowerDriveTrain(0);
    }


    public void turnLeft(double degrees, double power) {
        double robotCircumference = Math.PI * ROBOT_DIAMETER_INCHES;
        double distancePerDegree = robotCircumference / 360.0;
        double wheelDistanceForTurn = distancePerDegree * degrees;

        int moveCounts = (int) (wheelDistanceForTurn * COUNTS_PER_INCH);
        setDriveTargetPosition(-moveCounts, moveCounts, -moveCounts, moveCounts);
        setRunToPosition();
        setPowerDriveTrain(power);
        slowDownAtEnd(power);
        setPowerDriveTrain(0);

    }

    public void turnRight(double degrees, double power) {
        double robotCircumference = Math.PI * ROBOT_DIAMETER_INCHES;
        double distancePerDegree = robotCircumference / 360.0;
        double wheelDistanceForTurn = distancePerDegree * degrees;

        int moveCounts = (int) (wheelDistanceForTurn * COUNTS_PER_INCH);
        setDriveTargetPosition(moveCounts, -moveCounts, moveCounts, -moveCounts);
        setRunToPosition();
        setPowerDriveTrain(power);
        slowDownAtEnd(power);
        setPowerDriveTrain(0);
    }

}
