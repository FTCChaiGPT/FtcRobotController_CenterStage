package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="NoEncoderAutonomous", group = "Auto")
public class MecanumAutonomous extends LinearOpMode {

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize motors
        frontLeftMotor = hardwareMap.dcMotor.get("front_left_motor");
        frontRightMotor = hardwareMap.dcMotor.get("front_right_motor");
        rearLeftMotor = hardwareMap.dcMotor.get("rear_left_motor");
        rearRightMotor = hardwareMap.dcMotor.get("rear_right_motor");

        // Set motor directions (adjust as needed)
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart(); // Wait for the start button to be pressed

        // Autonomous code
        driveForward(0.5, 2000); // Drive forward at 50% power for 2 seconds
        stopDriving(); // Stop the robot
    }

    private void driveForward(double power, long duration) {
        runtime.reset();

        while (opModeIsActive() && runtime.milliseconds() < duration) {
            frontLeftMotor.setPower(power);
            frontRightMotor.setPower(power);
            rearLeftMotor.setPower(power);
            rearRightMotor.setPower(power);
        }
    }

    private void stopDriving() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
    }


    private void backwards(double power, long duration) {
        runtime.reset();

        while (opModeIsActive() && runtime.milliseconds() < duration) {
            frontLeftMotor.setPower(-power);
            frontRightMotor.setPower(-power);
            rearLeftMotor.setPower(-power);
            rearRightMotor.setPower(-power);
        }
    }

    private void turnLeft (double power, long duration) {
        runtime.reset();

        while (opModeIsActive() && runtime.milliseconds() < duration) {
            frontRightMotor.setPower(-power);
            rearLeftMotor.setPower(-power);

        }
    }

    private void turnRight(double power, long duration) {
        runtime.reset();

        while (opModeIsActive() && runtime.milliseconds() < duration) {
            frontLeftMotor.setPower(power);
            rearRightMotor.setPower(power);
        }
    }

    private void strafeRight(double power, long duration) {
        runtime.reset();

        while (opModeIsActive() && runtime.milliseconds() < duration) {
            frontLeftMotor.setPower(-power);
            frontRightMotor.setPower(power);
            rearLeftMotor.setPower(power);
            rearRightMotor.setPower(-power);
        }
    }

    private void strafeLeft(double power, long duration) {
        runtime.reset();

        while (opModeIsActive() && runtime.milliseconds() < duration) {
            frontLeftMotor.setPower(power);
            frontRightMotor.setPower(-power);
            rearLeftMotor.setPower(-power);
            rearRightMotor.setPower(power);
        }
    }



}

