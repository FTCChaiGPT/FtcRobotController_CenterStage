package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@Autonomous(name="GyroNew")
public class GryoTestUpdated extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         frontLeft   = null;
    private DcMotor         frontRight  = null;
    private DcMotor         backLeft   = null;
    private DcMotor         backRight  = null;
    private Servo wristServo = null;
    private IMU             imu        = null;      // Control/Expansion Hub IMU

    private double          headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double strafeSpeed = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
    private int     leftTarget    = 0;
    private int     rightTarget   = 0;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.775953;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.

    static final double TURN_SPEED = 0.2;
    static final double     STRAFE_SPEED            = 0.4;
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.01;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable





    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        frontLeft  = hardwareMap.get(DcMotor.class, "left_front");
        backLeft = hardwareMap.get(DcMotor.class, "left_back");
        frontRight = hardwareMap.get(DcMotor.class, "right_front");
        backRight = hardwareMap.get(DcMotor.class, "right_back");
        wristServo = hardwareMap.get(Servo.class, "wristservo");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
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
        imu.resetYaw();
        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            //imu.resetYaw();
            telemetry.update();
            wristServo.setPosition(0);
            sleep(500);
        }

        // Set the encoders for closed loop speed control, and reset the heading.
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu.resetYaw();

        // Step through each leg of the path,
        // Notes:   Reverse movement is obtained by setting a negative distance (not speed)
        //          holdHeading() is used after turns to let the heading stabilize
        //          Add a sleep(2000) after any step to keep the telemetry data visible for review



        driveStraight(DRIVE_SPEED,12.0, 0.0);
//        turnToHeading(MAX_TURN_SPEED, -90);
//        holdHeading(MAX_TURN_SPEED, -90, 0.5);
//        turnToHeading1(TURN_SPEED, -90);
//        holdHeading(TURN_SPEED, 0, 10);
        //strafe(STRAFE_SPEED, -12, 0.0);
        //strafe(STRAFE_SPEED, 12, 0.0);
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message.
        stopRobot();
    }

    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    public void driveStraight(double speed, double distance, double heading) {
        if (opModeIsActive()) {
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            resetEncoders();
            frontLeft.setTargetPosition(moveCounts);
            backLeft.setTargetPosition(moveCounts);
            frontRight.setTargetPosition(moveCounts);
            backRight.setTargetPosition(moveCounts);
            setMotorPower(speed);
            while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backRight.isBusy() && backLeft.isBusy()) {
            //Just wait for it to finish
            }
            }
    }
    public void driveStraight1(double speed, double distance, double heading) {

        if (opModeIsActive()) {
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            resetEncoders();

            // Set Target and switch to RUN_TO_POSITION
            setTargetPosition(moveCounts, moveCounts, moveCounts, moveCounts);

            setRunToPosition();

            speed = Math.abs(speed);
            setMotorPower(speed);
            //moveRobot(speed, 0, 0);
            // Drive until the target is reached
            while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backRight.isBusy() && backLeft.isBusy()) {
                int frontLeftPos = frontLeft.getCurrentPosition();
                int frontRightPos = frontRight.getCurrentPosition();
                telemetry.addData("Front Left current position", frontLeftPos);
                telemetry.addData("Front Right current position", frontRightPos);
                telemetry.update();
                //sleep(2000);
//                double correction = getSteeringCorrection(heading, P_DRIVE_GAIN);
//
//                if(distance < 0){
//                    turnSpeed *= -1.0;
//
//                }
//                moveRobot(speed, 0, turnSpeed);

                //slowDownAtEnd(speed);
                //sendTelemetry(true);
                sleep(5000);
            }

            // Stop and change mode back to RUN_USING_ENCODER
            moveRobot(0, 0, 0);
            setRunUsingEncoder();
        }
    }



    public void turnToHeading(double speed, double targetAngle) {

        telemetry.addData("targetAngle :: ", targetAngle);
        telemetry.update();
        telemetry.addData("heading :: ", getHeading());
        telemetry.update();
        /*while (opModeIsActive() && Math.abs(targetAngle - getHeading()) > HEADING_THRESHOLD) {

            double error = targetAngle - getHeading();
            double turnSpeed = getCorrection(targetAngle) * speed;

            // Scale down speed as you get closer to the target
            //turnSpeed *= (error / 180.0); // Assuming error is in degrees; scale proportionally
            turnSpeed = Math.signum(error)*Math.max(MIN_TURN_SPEED, Math.min(MAX_TURN_SPEED, Math.abs(turnSpeed * (error / 180.0))));
            // Ensure minimum speed for movement
            //turnSpeed = Math.max(turnSpeed, MIN_TURN_SPEED);
            telemetry.addData("turnSpeed :: ", turnSpeed);
            telemetry.update();
            telemetry.addData("heading :: ", getHeading());
            telemetry.update();
            moveRobot(0, 0, turnSpeed);

        }*/
        while (opModeIsActive() && !onTargetHeading(targetAngle)) {
            double error = getError(targetAngle);
            double turnSpeed = computeTurnSpeed(error, speed);

            telemetry.addData("turnSpeed :: ", turnSpeed);
            telemetry.update();
            telemetry.addData("heading :: ", getHeading());
            telemetry.update();
            moveRobot(0, 0, turnSpeed);
        }
        stopRobot();
    }

    private boolean onTargetHeading(double targetAngle) {
        return Math.abs(targetAngle - getHeading()) <= HEADING_THRESHOLD;
    }

    private double getError(double targetAngle) {
        double error = targetAngle - getHeading();
        // Normalize the error to be within +/- 180 degrees
        while (error > 180)  error -= 360;
        while (error <= -180) error += 360;
        return error;
    }

    private double computeTurnSpeed(double error, double maxSpeed) {
        final double MAX_TURN_SPEED = 0.9;     // Max Turn speed to limit turn rate

        final double MIN_TURN_SPEED = 0.3;
        double turnSpeed = Range.clip(error * P_TURN_GAIN, -1, 1);
        turnSpeed *= maxSpeed; // Scale with max speed
        turnSpeed = /*Math.signum(error) * */Math.max(MIN_TURN_SPEED, Math.min(MAX_TURN_SPEED, Math.abs(turnSpeed)));
        return turnSpeed;
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
            moveRobot(0, 0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        stopRobot();
    }
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }


    public void strafe(double speed, double distance, double heading) {
        int moveCounts = (int)(distance * COUNTS_PER_INCH);

        // Calculate target positions for strafing
        int leftFrontTarget = frontLeft.getCurrentPosition() + moveCounts;
        int leftRearTarget = backLeft.getCurrentPosition() - moveCounts;
        int rightFrontTarget = frontRight.getCurrentPosition() - moveCounts;
        int rightRearTarget = backRight.getCurrentPosition() + moveCounts;

        // Set Target and switch to RUN_TO_POSITION
        frontLeft.setTargetPosition(leftFrontTarget);
        backLeft.setTargetPosition(leftRearTarget);
        frontRight.setTargetPosition(rightFrontTarget);
        backRight.setTargetPosition(rightRearTarget);

        setRunToPosition();

        // Strafe until the target is reached
        while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backRight.isBusy() && backLeft.isBusy()) {
            double correction = getCorrection(heading);
            moveRobot(0, speed - correction, 0);
        }

        // Stop and change mode back to RUN_USING_ENCODER
        stopRobot();
        setRunUsingEncoder();
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */


    /**
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     *
     */
    public void moveRobot(double drive, double strafe, double rotate) {
        //double frontLeftSpeed = 1.75 * (drive + strafe + rotate);
        //double backLeftSpeed = 1.75 * (drive - strafe + rotate);

        double frontLeftSpeed = drive + strafe + rotate;
        double backLeftSpeed =  drive - strafe + rotate;

        double frontRightSpeed = drive - strafe - rotate;
        double backRightSpeed = drive + strafe - rotate;

        telemetry.addData("Left Front Speed", frontLeftSpeed);
        telemetry.addData("Right Front Speed", frontRightSpeed);
        telemetry.addData("Left Back Speed", backLeftSpeed);
        telemetry.addData("Right Back Speed", backRightSpeed);
        telemetry.update();
        sleep(5000);
        // Normalize the values so no speed exceeds 1.0
        double max = Math.max(Math.max(Math.abs(frontLeftSpeed), Math.abs(frontRightSpeed)),
                Math.max(Math.abs(backLeftSpeed), Math.abs(backRightSpeed)));
        if (max > 1.0) {
            frontLeftSpeed /= max;
            frontRightSpeed /= max;
            backLeftSpeed /= max;
            backRightSpeed /= max;
        }

        frontLeft.setPower(frontLeftSpeed);
        frontRight.setPower(frontRightSpeed);
        backLeft.setPower(backLeftSpeed);
        backRight.setPower(backRightSpeed);
    }

    public double getCorrection(double targetAngle) {
        double headingError = targetAngle - getHeading();
        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        return Range.clip(headingError * P_TURN_GAIN, -1, 1);
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
    public void stopRobot() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    public void setTargetPosition(int frontLeftTarget, int frontRightTarget, int backLeftTarget, int backRightTarget) {

        int frontLeftTarget_final = frontLeft.getCurrentPosition() + frontLeftTarget;
        frontLeft.setTargetPosition(frontLeftTarget_final);
        int frontRightTarget_final = frontRight.getCurrentPosition() + frontRightTarget;
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + frontRightTarget);
        int backLeftTarget_final = backLeft.getCurrentPosition() + backLeftTarget;
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + backLeftTarget);
        int backRightTarget_final = backRight.getCurrentPosition() + backRightTarget;
        backRight.setTargetPosition(backRight.getCurrentPosition() + backRightTarget);

        telemetry.addData("Front Left Target Position: ", frontLeftTarget_final);
        telemetry.addData("Front Right Target Position: ", frontRightTarget_final);
        telemetry.addData("Back Left Target Position: ", backLeftTarget_final);
        telemetry.addData("Back Right Target Position: ", backRightTarget_final);
    }
    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
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

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    public void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void slowDownAtEnd(double p) {
        while (opModeIsActive() && frontLeft.isBusy()) { //Accurate measures
            double remainingDistance = Math.abs(frontLeft.getTargetPosition() - frontLeft.getCurrentPosition());
            // Adjust this threshold as needed.
            double rampDownThreshold = 600;  // For example, start ramping down when 300 encoder counts away from target.

            if (remainingDistance < rampDownThreshold) {
                double rampedSpeed = 0.5 * (p * (remainingDistance / rampDownThreshold));
                rampedSpeed = Math.max(rampedSpeed, 0.1); // Ensure you have a minimum speed to prevent the robot from stalling.
                setMotorPower(rampedSpeed);
            }
        }
    }
    private void setMotorPower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }
}

//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//
//@Autonomous(name="MecanumIMUNavigate", group="Linear Opmode")
//public class GryoTest extends LinearOpMode {
//
//    // Motor declarations
//    private DcMotor frontLeft = null;
//    private DcMotor frontRight = null;
//    private DcMotor backLeft = null;
//    private DcMotor backRight = null;
//    private BNO055IMU imu;
//
//    // Constants for encoder drive
//    static final double COUNTS_PER_MOTOR_REV = 537.6; // for GoBILDA 5202 series
//    static final double DRIVE_GEAR_REDUCTION = 1.0;
//    static final double WHEEL_DIAMETER_INCHES = 4.0;
//    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//            (WHEEL_DIAMETER_INCHES * Math.PI);
//    static final double DRIVE_SPEED = 0.6;
//    static final double TURN_SPEED = 0.5;
//
//    @Override
//    public void runOpMode() {
//        // Motor hardware mapping
//        frontLeft = hardwareMap.get(DcMotor.class, "left_front");
//        frontRight = hardwareMap.get(DcMotor.class, "right_front");
//        backLeft = hardwareMap.get(DcMotor.class, "left_back");
//        backRight = hardwareMap.get(DcMotor.class, "right_back");
//
//        // IMU initialization
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);
//
//        // Wait for the start button to be pressed.
//        waitForStart();
//
//        // Autonomous actions
//        if (opModeIsActive()) {
//            encoderDrive(DRIVE_SPEED, 10, 10, 10, 10, 5.0); // Move 10 inches forward
////            turnUsingIMU(TURN_SPEED, 90);                   // Turn 90 degrees right
////            encoderDrive(DRIVE_SPEED, 15, 15, 15, 15, 5.0); // Move 15 inches forward
////            turnUsingIMU(TURN_SPEED, -120);                 // Turn 120 degrees left
//        }
//    }
//
//    // Method to perform a turn using the IMU
//    private void turnUsingIMU(double speed, double angle) {
//        Orientation startOrientation = imu.getAngularOrientation();
//        double targetAngle = startOrientation.firstAngle + angle;
//
//        while (opModeIsActive() && !onTargetAngle(targetAngle)) {
//            double turn = speed * Math.signum(angle);
//            moveRobot(0, turn, 0);
//        }
//
//        // Stop all motion
//        moveRobot(0, 0, 0);
//    }
//
//    // Method to drive straight for a certain distance using encoders
//    private void encoderDrive(double speed,
//                              double frontLeftInches, double frontRightInches,
//                              double backLeftInches, double backRightInches,
//                              double timeoutS) {
//        int newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget;
//
//        // Determine new target positions
//        newFrontLeftTarget = frontLeft.getCurrentPosition() + (int)(frontLeftInches * COUNTS_PER_INCH);
//        newFrontRightTarget = frontRight.getCurrentPosition() + (int)(frontRightInches * COUNTS_PER_INCH);
//        newBackLeftTarget = backLeft.getCurrentPosition() + (int)(backLeftInches * COUNTS_PER_INCH);
//        newBackRightTarget = backRight.getCurrentPosition() + (int)(backRightInches * COUNTS_PER_INCH);
//
//        // Set target positions for all motors
//        frontLeft.setTargetPosition(newFrontLeftTarget);
//        frontRight.setTargetPosition(newFrontRightTarget);
//        backLeft.setTargetPosition(newBackLeftTarget);
//        backRight.setTargetPosition(newBackRightTarget);
//
//        // Turn On RUN_TO_POSITION mode
//        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        // Start motion
//        moveRobot(speed, 0, 0);
//
//        // Keep looping while motors are running and timeout has not elapsed
//        while (opModeIsActive() &&
//                (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {
//            // Wait for completion
//        }
//
//        // Stop all motion
//        moveRobot(0, 0, 0);
//
//        // Turn off RUN_TO_POSITION mode
//        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    // Check if the robot is at the target angle
//    private boolean onTargetAngle(double targetAngle) {
//        Orientation angles = imu.getAngularOrientation();
//        double heading = angles.firstAngle;
//        return Math.abs(heading - targetAngle) < 1; // Using a tolerance of 1 degree
//    }
//
//    // Method to control all four motors for mecanum wheel movement
//    private void moveRobot(double drive, double turn, double strafe) {
//        double frontLeftSpeed = drive + turn + strafe;
//        double frontRightSpeed = drive - turn - strafe;
//        double backLeftSpeed = drive + turn - strafe;
//        double backRightSpeed = drive - turn + strafe;
//
//        // Normalize the values so no speed exceeds 1.0
//        double max = Math.max(Math.max(Math.abs(frontLeftSpeed), Math.abs(frontRightSpeed)),
//                Math.max(Math.abs(backLeftSpeed), Math.abs(backRightSpeed)));
//        if (max > 1.0) {
//            frontLeftSpeed /= max;
//            frontRightSpeed /= max;
//            backLeftSpeed /= max;
//            backRightSpeed /= max;
//        }
//
//        frontLeft.setPower(frontLeftSpeed);
//        frontRight.setPower(frontRightSpeed);
//        backLeft.setPower(backLeftSpeed);
//        backRight.setPower(backRightSpeed);
//    }
//}