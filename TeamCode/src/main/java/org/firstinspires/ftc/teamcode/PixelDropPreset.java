package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

/**
 *Code 1 motor such that -
 *
 * Once button x is pressed:
 * 220 degree/angle set position
 *
 * With y axis of left joystick-
 * you can control the motor to move up and down from the set position of 220 degrees.
 *
 * Catch: there is 235 degrees limit
 */
public class PixelDropPreset extends OpMode {
    private DcMotor linearSlide;
    private DcMotor linearSlide2;
    private Servo arm;
    private Servo arm2;
    private Gamepad gamepad1;
    private int curr_pos;
    final private int MOTOR_DEFAULT_POS = 220;



    int maxArmPosition = 1000;//(measured distance in inches) * (encoder counts per inch)


    private static final double COUNTS_PER_MOTOR_REV = 5281;

    boolean prev_a;
    boolean curr_a;
    @Override
    public void init() {
        linearSlide = hardwareMap.get(DcMotor.class, "linearSlide");
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide2 = hardwareMap.get(DcMotor.class, "linearSlide2");
        linearSlide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm = hardwareMap.get(Servo.class, "arm");
        arm2 = hardwareMap.get(Servo.class, "arm2");
        curr_pos = linearSlide.getCurrentPosition();
    }
    @Override
    public void loop() {
        prev_a = curr_a;
        curr_a = gamepad1.x;
        if (curr_a && !prev_a){
            turnMotor(MOTOR_DEFAULT_POS);
            arm.setPosition(0.31);
            arm2.setPosition(0.31);
        }
        if (gamepad1.left_stick_y > 0){
            linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            linearSlide.setDirection(DcMotor.Direction.FORWARD);
            linearSlide.setPower(0.5);
            linearSlide.setTargetPosition((int) (gamepad1.left_stick_y * maxArmPosition));
            linearSlide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            linearSlide2.setDirection(DcMotor.Direction.FORWARD);
            linearSlide2.setPower(0.5);
            linearSlide2.setTargetPosition((int) (gamepad1.left_stick_y * maxArmPosition));
            curr_pos = linearSlide.getCurrentPosition();
        }
        if (gamepad1.left_stick_y < 0){
            linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            linearSlide.setDirection(DcMotor.Direction.REVERSE);
            linearSlide.setPower(0.5);
            linearSlide.setTargetPosition((int) (gamepad1.left_stick_y * maxArmPosition));
            linearSlide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            linearSlide2.setDirection(DcMotor.Direction.REVERSE);
            linearSlide2.setPower(0.5);
            linearSlide2.setTargetPosition((int) (gamepad1.left_stick_y * maxArmPosition));
            curr_pos = linearSlide.getCurrentPosition();
        }
        if(linearSlide2.getCurrentPosition() != linearSlide.getCurrentPosition()){
            linearSlide2.setTargetPosition((int)((COUNTS_PER_MOTOR_REV/360)*(linearSlide2.getCurrentPosition()-curr_pos)));
            linearSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(linearSlide2.isBusy()){
                int error = (linearSlide2.getCurrentPosition()-curr_pos) - curr_pos;
                int power = (int)(error * 0.01);
                setMotorPower(power);
            }
        }
        /*if (gamepad1.left_stick_y == 0 || curr_pos > ((COUNTS_PER_MOTOR_REV/360) * 234.9) || curr_pos < ((COUNTS_PER_MOTOR_REV/360) * 0.1)){
            linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            linearSlide.setPower(0);
        }*/
    }
    public void turnMotor(int degrees) {

        linearSlide.setTargetPosition((int)((COUNTS_PER_MOTOR_REV/360)*degrees));
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide2.setTargetPosition((int)((COUNTS_PER_MOTOR_REV/360)*degrees));
        linearSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        curr_pos = linearSlide.getCurrentPosition();

        while(linearSlide.isBusy() && linearSlide2.isBusy()){
            int error = degrees - curr_pos;
            int power = (int)(error * 0.01);
            setMotorPower(power);
        }
        stopMotor();
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setMotorPower(double x) {
        linearSlide.setPower(x);
        linearSlide2.setPower(x);
    }
    public void stopMotor() {
        setMotorPower(0);
    }


}
