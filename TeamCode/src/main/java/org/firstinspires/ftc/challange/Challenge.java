package org.firstinspires.ftc.challange;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Challenge extends OpMode {

    private DcMotor motor;
    boolean toggle = false;
    private static final double COUNTS_PER_MOTOR_REV = 5281; //Number of encoder counts per motor revolution (1440)
    private static final double WHEEL_DIAMETER_INCHES = 3.75;
    private  static final double GEAR_RATIO = 1.0;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * GEAR_RATIO)/(WHEEL_DIAMETER_INCHES * Math.PI);

    private static final double ROBOT_WIDTH_INCHES = 18.9; // The distance between the wheels on opposite sides

    boolean prev_a;
    boolean curr_a;


    private Servo servo1;
    private Servo servo2;
    private CRServo servo3;
    private CRServo servo4;
    private Gamepad gamepad1;
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo3 = hardwareMap.get(CRServo.class, "servo3");
        servo4 = hardwareMap.get(CRServo.class, "servo4");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        prev_a = curr_a;
        curr_a = gamepad1.a;
        if (curr_a && prev_a){//hold the 'a' button
            motor.setPower(1);
            servo3.setPower(1);
            servo4.setPower(1);
        }
        else if (curr_a && !prev_a && !toggle) {//clicking the 'a' button
            turnMotor(180, 0.5);
            servo1.setPosition(0.995);
            servo2.setPosition(0.995);
            toggle = true;
        } else if (curr_a && !prev_a && toggle) {//clicking the 'a' button for the second time
            turnMotor(90, 0.5);
            servo1.setPosition(0.5);
            servo2.setPosition(0.5);
            toggle = false;
        }
    }
    public void turnMotor(int distance, double power) {
        resetEncoders();

        int target = (int) (distance * COUNTS_PER_INCH);
        motor.setTargetPosition(-target);

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        setMotorPower(power);



        stopMotor();
    }

    private void setMotorPower(double x) {
        motor.setPower(x);
    }
    public void stopMotor() {
        setMotorPower(0);
        resetEncoders();
    }

    private void resetEncoders() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

}
