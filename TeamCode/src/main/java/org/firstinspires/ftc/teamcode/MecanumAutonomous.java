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

    private DcMotor intake;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize motors
        frontLeftMotor = hardwareMap.dcMotor.get("left_front");
        frontRightMotor = hardwareMap.dcMotor.get("right_front");
        rearLeftMotor = hardwareMap.dcMotor.get("left_back");
        rearRightMotor = hardwareMap.dcMotor.get("right_back");

        // Set motor directions (adjust as needed)f
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart(); // Wait for the start button to be pressed



        // Autonomous code
        driveForward(1, 2000);
        //turnRight(1, 90);
        //driveForward(0.5, 200);
        //stopDriving();

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

