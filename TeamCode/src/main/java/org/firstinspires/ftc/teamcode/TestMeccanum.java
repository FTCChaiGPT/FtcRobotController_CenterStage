package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "TestMeccanum")
public class TestMeccanum extends LinearOpMode {
    private DcMotor leftFront_motor;

    private DcMotor leftBack_motor;
    private DcMotor rightFront_motor;

    private DcMotor rightBack_motor;
    private static final double COUNTS_PER_MOTOR_REV_DRIVE = 537;
    private static final double WHEEL_DIAMETER_INCHES = 3.78;
    private static final double GEAR_RATIO = 1.0;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV_DRIVE * GEAR_RATIO) / (WHEEL_DIAMETER_INCHES * Math.PI);
    private static final double ROBOT_WIDTH_INCHES = 18.9; // The distance between the wheels on opposite sides (diagonal)


    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        leftFront_motor = hardwareMap.get(DcMotor.class, "left_front");
        leftBack_motor = hardwareMap.get(DcMotor.class, "left_back");
        rightFront_motor = hardwareMap.get(DcMotor.class, "right_front");
        rightBack_motor = hardwareMap.get(DcMotor.class, "right_back");

        forward(10,0.5);
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
}


