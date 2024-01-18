package org.firstinspires.ftc.challange;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "PIDControl")

public class PIDcontrol extends LinearOpMode {
    DcMotorEx leftFront;

    DcMotorEx rightFront;
    DcMotorEx leftBack;
    DcMotorEx rightBack;
    private static final double COUNTS_PER_MOTOR_REV = 751.8; //Number of encoder counts per motor revolution
    private static final double WHEEL_DIAMETER_INCHES = 5.5;
    private static final double GEAR_RATIO = 1.0;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * GEAR_RATIO)/(WHEEL_DIAMETER_INCHES * Math.PI);


    double integralSumLF = 0, integralSumRF = 0,  integralSumLB = 0, integralSumRB = 0;
    double Kp = 0.01;
    double Ki = 0;
    double Kd = 0;
    ElapsedTime timer = new ElapsedTime();
    private double lastErrorLF = 0, lastErrorRF = 0, lastErrorLB = 0, lastErrorRB = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()) {
            forward(12);

        }

    }
    public double PIDControl(double reference, double state, double integralSum, double lastError){
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();

        double output = (error * Kp) + (integralSum * Ki) + (derivative * Kd);
        return clamp(output, -1, 1);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }

    public void forward(double inches) {
        move(-0.25, 0, convertInchesToEncoderCounts(inches));
    }

    public void backward(double inches) {
        move(0.995, 0, convertInchesToEncoderCounts(inches));
    }

    public void strafe(int inches) {
        move(0, -0.995, (int) (inches* COUNTS_PER_INCH));
    }

    public void turn(int inches) {
        move(0, 0, (int) (inches* COUNTS_PER_INCH));
    }

    private int convertInchesToEncoderCounts(double inches) {
        return (int) (inches * COUNTS_PER_INCH);
    }


    private void move(double forward, double strafe, int encoderCounts) {
        double targetPositionLF = leftFront.getCurrentPosition() + encoderCounts;
        double targetPositionRF = rightFront.getCurrentPosition() + encoderCounts;
        double targetPositionLB = leftBack.getCurrentPosition() + encoderCounts;
        double targetPositionRB = rightBack.getCurrentPosition() + encoderCounts;

        telemetry.addData("LF Pos", leftFront.getCurrentPosition());
        telemetry.addData("RF Pos", rightFront.getCurrentPosition());
        telemetry.addData("LB Pos", leftBack.getCurrentPosition());
        telemetry.addData("RB Pos", rightBack.getCurrentPosition());
        telemetry.addData("LF Target", targetPositionLF);
        telemetry.addData("RF Target", targetPositionLB);
        telemetry.addData("LB Target", targetPositionRF);
        telemetry.addData("RB Target", targetPositionRB);
        telemetry.update();

        double powerLF, powerLB, powerRF, powerRB;



        do{
            powerLF = PIDControl(targetPositionLF, leftFront.getCurrentPosition(), integralSumLF, lastErrorLF);
            powerLB = PIDControl(targetPositionLB, leftBack.getCurrentPosition(), integralSumRF, lastErrorRF);
            powerRF = PIDControl(targetPositionRF, rightFront.getCurrentPosition(), integralSumLB, lastErrorLB);
            powerRB = PIDControl(targetPositionRB, rightBack.getCurrentPosition(), integralSumRB, lastErrorRB);

            leftFront.setPower(forward + strafe + powerLF);
            rightFront.setPower(forward - strafe + powerRF);
            leftBack.setPower(forward - strafe + powerLB);
            rightBack.setPower(forward + strafe + powerRB);

            telemetry.addData("Current LF Pos", leftFront.getCurrentPosition());
            telemetry.addData("Current RF Pos", rightFront.getCurrentPosition());
            telemetry.addData("Current LB Pos", leftBack.getCurrentPosition());
            telemetry.addData("Current RB Pos", rightBack.getCurrentPosition());
            telemetry.addData("LF Power", powerLF);
            telemetry.addData("RF Power", powerRF);
            telemetry.addData("LB Power", powerLB);
            telemetry.addData("RB Power", powerRB);
            telemetry.update();
        }

        while (opModeIsActive() && (Math.abs(leftFront.getCurrentPosition() - targetPositionLF) > 500 || Math.abs(rightFront.getCurrentPosition() - targetPositionLB) > 500 || Math.abs(leftBack.getCurrentPosition() - targetPositionRF) > 500 || Math.abs(rightBack.getCurrentPosition() - targetPositionRB) > 500));

        telemetry.addData("Final LF Pos", leftFront.getCurrentPosition());
        telemetry.addData("Final RF Pos", rightFront.getCurrentPosition());
        telemetry.addData("Final LB Pos", leftBack.getCurrentPosition());
        telemetry.addData("Final RB Pos", rightBack.getCurrentPosition());
        telemetry.update();

        updateErrorsAndSums(targetPositionLF, targetPositionRF, targetPositionLB, targetPositionRB);
        stopMotors();
    }

    private void updateErrorsAndSums(double targetPositionLF, double targetPositionRF, double targetPositionLB, double targetPositionRB) {
        lastErrorLF = targetPositionLF - leftFront.getCurrentPosition();
        lastErrorRF = targetPositionRF - rightFront.getCurrentPosition();
        lastErrorLB = targetPositionLB - leftBack.getCurrentPosition();
        lastErrorRB = targetPositionRB - rightBack.getCurrentPosition();

        integralSumLF += lastErrorLF * timer.seconds();
        integralSumRF += lastErrorRF * timer.seconds();
        integralSumLB += lastErrorLB * timer.seconds();
        integralSumRB += lastErrorRB * timer.seconds();
        timer.reset();
    }


    private void stopMotors() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

}
