package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="MeccanumAutoEncoder", group = "Auto")
public class MecanumEncoderAutonomous extends LinearOpMode {

    private DcMotor left_front;
    private DcMotor left_back;
    private DcMotor right_front;
    private DcMotor right_back;

    private static final double COUNTS_PER_MOTOR_REV = 1440; // Number of encoder counts per motor revolution
    private static final double WHEEL_DIAMETER_INCHES = 4.0; // Diameter of the wheel in inches
    private static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        right_back = hardwareMap.get(DcMotor.class, "right_back");
        right_back.setDirection(DcMotor.Direction.REVERSE);
        left_back.setDirection(DcMotor.Direction.REVERSE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        backward(3, 0.5);
        strafe_Right(12, 1);


    }



    public void forward(int distance, double power) {

        int target = (int) (distance * COUNTS_PER_INCH);
        left_front.setTargetPosition(target);
        left_back.setTargetPosition(target);
        right_front.setTargetPosition(target);
        right_back.setTargetPosition(target);

        left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        setMotorPower(power);

        stopMotor();

    }

    private void setMotorPower(double x) {
        left_front.setPower(x);
        left_back.setPower(x);
        right_front.setPower(x);
        right_back.setPower(x);
    }

    public void backward(int distance, double power) {
        int target = (int) (distance * COUNTS_PER_INCH);
        left_front.setTargetPosition(-target);
        left_back.setTargetPosition(-target);
        right_front.setTargetPosition(-target);
        right_back.setTargetPosition(-target);

        left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        setMotorPower(power);

        stopMotor();

    }

    public void turn_Left(int distance, double power) {
        int target = (int) (distance * COUNTS_PER_INCH);
        left_back.setTargetPosition(-target);
        right_front.setTargetPosition(-target);

        left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        setMotorPower(power);

        stopMotor();

    }

    public void turn_Right(int distance, double power) {
        int target = (int) (distance * COUNTS_PER_INCH);
        left_front.setTargetPosition(target);
        right_back.setTargetPosition(target);

        left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        setMotorPower(power);

        stopMotor();

    }

    public void strafe_Right(int distance, double power) {
        int target = (int) (distance * COUNTS_PER_INCH);
        left_front.setTargetPosition(-target);
        left_back.setTargetPosition(target);
        right_front.setTargetPosition(target);
        right_back.setTargetPosition(-target);

        left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        setMotorPower(power);

        stopMotor();

    }

    public void strafe_Left(int distance, double power) {
        int target = (int) (distance * COUNTS_PER_INCH);
        left_front.setTargetPosition(target);
        left_back.setTargetPosition(-target);
        right_front.setTargetPosition(-target);
        right_back.setTargetPosition(target);

        left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setMotorPower(power);

        stopMotor();

    }

    public void stopMotor() {
        setMotorPower(0);
        resetEncoders();
    }

    private void resetEncoders() {
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }


}

