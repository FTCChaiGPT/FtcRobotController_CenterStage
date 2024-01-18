package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


public class TestDrive extends LinearOpMode {

    private DcMotor arm;

    private static final double COUNTS_PER_MOTOR_REV = 750; //Number of encoder counts per motor revolution (1440)
    private static final double WHEEL_DIAMETER_INCHES = 3.75;
    private  static final double GEAR_RATIO = 1.0;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * GEAR_RATIO)/(WHEEL_DIAMETER_INCHES * Math.PI);

    private static final double ROBOT_WIDTH_INCHES = 18.9; // The distance between the wheels on opposite sides

    private final double MAX_POSITION = 220.0/270.0; // normalized for 270 degrees

    public void Arm(int degrees, double power) {

        if (gamepad1.x) {
            resetEncoders();

            double radians = Math.toRadians(degrees);
            double wheelDistance = radians * (ROBOT_WIDTH_INCHES / 2);

            int encoderCounts = (int) (wheelDistance * COUNTS_PER_INCH);

            // Set the target position for a left turn
            arm.setTargetPosition(-encoderCounts);

            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Power the motors to move towards the target position
            setMotorPower(power);


            stopMotor();
        }
        else{
            stopMotor();
        }
    }

    private void resetEncoders() {
       arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private void setMotorPower(double x) {
        arm.setPower(x);

    }

    public void stopMotor() {
        setMotorPower(0);
        resetEncoders();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        arm = hardwareMap.get(DcMotor.class, "arm");

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Arm(180, 1);

    }
}
