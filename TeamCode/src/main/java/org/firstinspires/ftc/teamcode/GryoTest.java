package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.GryoTest.PropPostition.CENTER;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@Autonomous(name="GyroNewTest")
public class GryoTest extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor     frontLeft   = null;
    private DcMotor     frontRight  = null;
    private DcMotor     backLeft   = null;
    private DcMotor     backRight  = null;
    private DcMotor     intake     = null;
    private Servo       wristServo = null;
    private IMU         imu        = null;      // Control/Expansion Hub IMU
    private double      headingError  = 0;

    public enum PropPostition {
        LEFT, CENTER, RIGHT, UNKNOWN

    }
    public PropPostition spikePosition;

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

    private ElapsedTime timer;
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
    static final double     DRIVE_SPEED = 0.4;     // Max driving speed for better distance accuracy.

    static final double     TURN_SPEED = 0.2;
    static final double     STRAFE_SPEED  = 0.6;
    static final double     HEADING_THRESHOLD  = 1.0 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN  = 0.002;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN  = 0.03;     // Larger is more responsive, but also less stable

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        DriveInitialization();
        while (opModeInInit()) {
            targetHeading = getHeading();
            telemetry.addData(">", "Robot Heading = %4.0f", targetHeading);
            telemetry.update();
            wristServo.setPosition(0);
        }
        waitForStart();
        spikePosition = PropPostition.LEFT;
        purplePixelDrop(spikePosition);



//        driveStraight(DRIVE_SPEED,24.0, targetHeading);  // Drive Forward 10"
//        telemetry.addData("distance", "Complete");
//        telemetry.update();
//        turnToHeading( TURN_SPEED, 90.0); // Turn  CW to -45 Degrees (+ve value turns left, -ve turns right)
//        telemetry.addData("turn", "Complete");
//        telemetry.update();
//        holdHeading( TURN_SPEED, 90.0, 0.5);
//        telemetry.addData("HoldTurn", "Complete");
//        telemetry.update();
//        driveStraight(DRIVE_SPEED,-10.0, targetHeading);  // Drive Forward 10"
//        telemetry.addData("distance2", "Complete");
//        telemetry.update();

        // Step through each leg of the path,
        // Notes:   Reverse movement is obtained by setting a negative distance (not speed)
        //          holdHeading() is used after turns to let the heading stabilize
        //          Add a sleep(2000) after any step to keep the telemetry data visible for review

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
     * Initialize everything.
     * ====================================================================================================
     */

    public void purplePixelDrop(PropPostition spikePosition){
        PropPostition spikePosition1 = spikePosition;
        switch (spikePosition1) {
            case LEFT:
                //left logic
                telemetry.addLine("LEFT, going for LEFT purple pixel drop");
                telemetry.update();
                driveStraight(DRIVE_SPEED, 30, 0);
                turnToHeading(TURN_SPEED, 90);
                holdHeading(TURN_SPEED, 90, 0.5);
                driveStraight(DRIVE_SPEED, 2, 0);
                intake.setPower(0.2);
                sleep(500);
                driveStraight(DRIVE_SPEED, -3, 0);
                intake.setPower(0);
                driveStraight(DRIVE_SPEED, -8, 0);

                break;

            case CENTER:
                //center logic
                telemetry.addLine("CENTER, going for CENTER purple pixel drop");
                telemetry.update();
                driveStraight(DRIVE_SPEED, 31, 0);
                intake.setPower(0.2);
                sleep(500);
                driveStraight(DRIVE_SPEED, -3, 0);
                intake.setPower(0);
                driveStraight(DRIVE_SPEED, -8, 0);
                break;

            case RIGHT:
                //right logic
                telemetry.addLine("RIGHT, going for RIGHT purple pixel drop");
                telemetry.update();
                driveStraight(DRIVE_SPEED, 2, 0);
                strafe(STRAFE_SPEED, 15.75F, 0);
                turnToHeading(TURN_SPEED, 0);
                holdHeading(TURN_SPEED, 0, 0.5);
                driveStraight(DRIVE_SPEED, 20, 0);
                intake.setPower(0.2);
                sleep(500);
                driveStraight(DRIVE_SPEED, -3, 0);
                intake.setPower(0);
                driveStraight(DRIVE_SPEED, -8, 0);




                break;

            case UNKNOWN:
                //unknown logic
                telemetry.addLine("UNKNOWN, so trial and error on center logic");

                break;

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

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(DcMotor.Direction.FORWARD);

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
        // Set the encoders for closed loop speed control, and reset the heading.
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu.resetYaw();
    }

    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    public void driveStraight(double maxDriveSpeed, double distance, double heading) {
        setRunUsingEncoder();
        resetEncoders();

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
        resetEncoders();
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
        sleep(1000);

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

    public void resetEncoders() {
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
        telemetry.addData("GetHeading :", "%5.2f", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.update();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

}