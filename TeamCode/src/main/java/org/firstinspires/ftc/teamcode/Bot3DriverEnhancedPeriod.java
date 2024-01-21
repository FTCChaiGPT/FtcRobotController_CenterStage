//package org.firstinspires.ftc.teamcode;
//
//import static java.lang.Thread.sleep;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//@TeleOp (name = "Bot3DriverEnhancedPeriod", group = "iterativeOpMode")
//public class Bot3DriverEnhancedPeriod extends OpMode {
//
//    private DcMotor left_front_motor;//drive
//    private DcMotor left_back_motor;//drive
//    private DcMotor right_front_motor;//drive
//    private DcMotor right_back_motor;//drive
//    private Servo drone_servo;
//    private boolean prev_gamepad1rightbumper;
//    private boolean cur_gamepad1rightbumper;
//    private boolean prev_gamepad2b;
//    private boolean cur_gamepad2b;
//    private boolean prev_gamepad2rb;
//    private boolean cur_gamepad2rb;
//    private boolean prev_gamepad2lb;
//    private boolean cur_gamepad2lb;
//    private boolean cur_gamepad2dpad_l;
//    private boolean prev_gamepad2dpad_l;
//
//    private boolean cur_gamepad2dpad_r;
//    private boolean prev_gamepad2dpad_r;
//    private DcMotor intake_motor;
//    private CRServo right_intake_servo;
//    private CRServo left_intake_servo;
//    private Servo wristServo_servo;
//    private DcMotor rightLinearSlide_motor;
//    private DcMotor leftLinearSlide_motor;
//    //private double manualControlLinearSlidePowerValue = 0.35;
//    private double linearSlidePowerValue = 0.995;
//    private static final double COUNTS_PER_MOTOR_REV_ARM = 1440;
//    private static final double LINEAR_SLIDES_DEGREES_PER_TIC = COUNTS_PER_MOTOR_REV_ARM / 360;
//    private int linear_slides_tic_value;
//    private double speed;
//    private double DRIVE_MAX_SPEED = 1;
//    private double DRIVE_MIN_SPEED = 0.25;
//    private int LinearSlidePresetOne = 500; //scoring position
//    private int LinearSlidePresetTwo = 0; //intake position
//
//    ElapsedTime presetOneTimer;
//    private int WristPresetOne =  50/270; //scoring position
//    private int WristPresetTwo = 0; //intake position
//    @Override
//    public void init() {
//        left_front_motor = hardwareMap.get(DcMotor.class, "left_front");
//        left_back_motor = hardwareMap.get(DcMotor.class, "left_back");
//        right_front_motor = hardwareMap.get(DcMotor.class, "right_front");
//        right_back_motor = hardwareMap.get(DcMotor.class, "right_back");
//
//        //telling motors to stop with a break
//        //~drive~
//        left_front_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        left_back_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        right_front_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        right_back_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        intake_motor = hardwareMap.get(DcMotor.class, "intake");
//        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        intake_motor.setDirection(DcMotor.Direction.REVERSE);
//        intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        right_intake_servo = hardwareMap.get(CRServo.class, "rightwristservo");
//        right_intake_servo.setDirection(CRServo.Direction.REVERSE);
//        left_intake_servo = hardwareMap.get(CRServo.class, "leftwristservo");
//        drone_servo = hardwareMap.get(Servo.class, "drone");
//        wristServo_servo = hardwareMap.servo.get("wristservo");
//
//        rightLinearSlide_motor = hardwareMap.get(DcMotor.class, "rightlinearslide");
//        leftLinearSlide_motor = hardwareMap.get(DcMotor.class, "leftlinearslide");
//        rightLinearSlide_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftLinearSlide_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightLinearSlide_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftLinearSlide_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        presetOneTimer = new ElapsedTime();
//
//    }
//
//    public void reset_encoders() {
//        rightLinearSlide_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftLinearSlide_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
//
//    public void LinearSlidesMoveToPosition(int degrees) {
//        reset_encoders();
//        presetOneTimer.reset();
//        rightLinearSlide_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftLinearSlide_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        linear_slides_tic_value = (int) Math.round(degrees * LINEAR_SLIDES_DEGREES_PER_TIC);
//        rightLinearSlide_motor.setTargetPosition(linear_slides_tic_value);
//        leftLinearSlide_motor.setTargetPosition(linear_slides_tic_value);
//        rightLinearSlide_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftLinearSlide_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightLinearSlide_motor.setPower(linearSlidePowerValue);
//        leftLinearSlide_motor.setPower(linearSlidePowerValue);
//        // wait for motors to reach target position
//        while (leftLinearSlide_motor.isBusy() && rightLinearSlide_motor.isBusy() && presetOneTimer.seconds() < 5) {
//            // Add telemetry data here to monitor your motor's status
//
//        }
//        // Stop motors once target is reached
//        leftLinearSlide_motor.setPower(0);
//        rightLinearSlide_motor.setPower(0);
//    }
//
//    public void start() {
//        reset_encoders();
//    }
//
//    public void Drivetrain() {
//        if (cur_gamepad1rightbumper && !prev_gamepad1rightbumper) {
//            speed = DRIVE_MIN_SPEED;
//        } else {
//            speed = DRIVE_MAX_SPEED;
//        }
//
//
//        //has max power of 1 so multiplying it by 0.25 will change its max to 0.25, moving it 4x slower
//        left_front_motor.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x) * speed);
//        left_back_motor.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x) * speed);
//        right_front_motor.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x) * speed);
//        right_back_motor.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x) * speed);
//
//    }
//    public void booleanKeys() {
//        prev_gamepad1rightbumper = cur_gamepad1rightbumper;
//        cur_gamepad1rightbumper = gamepad1.right_bumper;
//        prev_gamepad2b = cur_gamepad2b;
//        cur_gamepad2b = gamepad2.b;
//        prev_gamepad2rb = cur_gamepad2rb;
//        cur_gamepad2rb = gamepad2.right_bumper;
//        prev_gamepad2lb = cur_gamepad2lb;
//        cur_gamepad2lb = gamepad2.left_bumper;
//        prev_gamepad2dpad_l = cur_gamepad2dpad_l;
//        cur_gamepad2dpad_l = gamepad2.dpad_left;
//        prev_gamepad2dpad_r = cur_gamepad2dpad_r;
//        cur_gamepad2dpad_r = gamepad2.dpad_right;
//    }
//
//    public void intakeAndOutake() {
//        if (gamepad2.right_trigger > 0.08) {
//            right_intake_servo.setPower(0.995);
//            left_intake_servo.setPower(0.995);
//            intake_motor.setPower(0.995);
//        }
//        else if (cur_gamepad2b && prev_gamepad2b) {
//            right_intake_servo.setPower(-0.995);
//            left_intake_servo.setPower(-0.995);
//            intake_motor.setPower(-0.995);
//        }
//        else {
//            right_intake_servo.setPower(0);
//            left_intake_servo.setPower(0);
//            intake_motor.setPower(0);
//        }
//
//        if(cur_gamepad2dpad_r && prev_gamepad2dpad_r){
//            right_intake_servo.setPower(-0.995);
//        }
//        else{
//            right_intake_servo.setPower(0);
//        }
//
//        if(cur_gamepad2dpad_l && prev_gamepad2dpad_l){
//            left_intake_servo.setPower(-0.995);
//        }
//        else{
//            left_intake_servo.setPower(0);
//        }
//
//        if(gamepad2.dpad_up){
//            drone_servo.setPosition(0.95);
//        }
//        else{
//            drone_servo.setPosition(0);
//        }
//
//
//        if (cur_gamepad2lb && !prev_gamepad2lb) {// preset 2 - intake position
//            wristServo_servo.setPosition(WristPresetTwo);
//            LinearSlidesMoveToPosition(LinearSlidePresetTwo);
//        }
//        else if (cur_gamepad2rb && !prev_gamepad2rb) {// preset 1 - scoring position
//            LinearSlidesMoveToPosition(LinearSlidePresetOne);
//            wristServo_servo.setPosition(WristPresetOne);
//        }
//        else if (Math.abs(gamepad2.left_stick_y) > 0.02) {// manual control
//            //Manual linear slide control
//            rightLinearSlide_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            leftLinearSlide_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            if (gamepad2.left_stick_y > 0.02) {
//
//                rightLinearSlide_motor.setPower(0.5);
//                leftLinearSlide_motor.setPower(0.5);
//            }
//            else if (gamepad2.left_stick_y < 0.02) {
//                rightLinearSlide_motor.setPower(-0.5);
//                leftLinearSlide_motor.setPower(-0.5);
//            }
//
//        }
//        else if (Math.abs(gamepad2.right_stick_y) > 0.02){
//            //Manual wrist control
//            if (gamepad2.right_stick_y > 0.02) {
//                wristServo_servo.setPosition(wristServo_servo.getPosition() + 0.2);
//            }
//            else if (gamepad2.right_stick_y < 0.02) {
//                wristServo_servo.setPosition(wristServo_servo.getPosition() - 0.2);
//            }
//        }
//        else{
//            rightLinearSlide_motor.setPower(0);
//            leftLinearSlide_motor.setPower(0);
//        }
//
//        //outtake
//        if (gamepad2.left_trigger > 0.08) {
//            right_intake_servo.setPower(-0.995);
//            left_intake_servo.setPower(-0.995);
//        }
//        else {
//            right_intake_servo.setPower(0);
//            left_intake_servo.setPower(0);
//        }
//    }
//
//    @Override
//    public void loop() {
//        booleanKeys();
//        Drivetrain();
//        intakeAndOutake();
//    }
//}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Bot3DriverEnhancedPeriod", group = "iterativeOpMode")
public class Bot3DriverEnhancedPeriod extends OpMode {

    private DcMotor left_front_motor;//drive
    private DcMotor left_back_motor;//drive
    private DcMotor right_front_motor;//drive
    private DcMotor right_back_motor;//drive
    private Servo drone_servo;
    private boolean prev_gamepad1rightbumper;
    private boolean cur_gamepad1rightbumper;
    private boolean prev_gamepad2b;
    private boolean cur_gamepad2b;
    private boolean prev_gamepad2rb;
    private boolean cur_gamepad2rb;
    private boolean prev_gamepad2lb;
    private boolean cur_gamepad2lb;
    private boolean cur_gamepad2dpad_l;
    private boolean prev_gamepad2dpad_l;

    private boolean cur_gamepad2dpad_r;
    private boolean prev_gamepad2dpad_r;
    private DcMotor intake_motor;
    private CRServo right_intake_servo;
    private CRServo left_intake_servo;
    private Servo wristServo_servo;
    private DcMotor rightLinearSlide_motor;
    private DcMotor leftLinearSlide_motor;
    //private double manualControlLinearSlidePowerValue = 0.35;
    private double linearSlidePowerValue = 0.995;
    private static final double COUNTS_PER_MOTOR_REV_ARM = 1440;
    private static final double LINEAR_SLIDES_DEGREES_PER_TIC = COUNTS_PER_MOTOR_REV_ARM / 360;
    private int linear_slides_tic_value;
    private double speed;
    private double DRIVE_MAX_SPEED = 1;
    private double DRIVE_MIN_SPEED = 0.25;
    private int LinearSlidePresetOne = 500; //scoring position
    private int LinearSlidePresetTwo = 0; //intake position
    private int WristPresetOne =  50/270; //scoring position
    private int WristPresetTwo = 0; //intake position
    @Override
    public void init() {
        left_front_motor = hardwareMap.get(DcMotor.class, "left_front");
        left_back_motor = hardwareMap.get(DcMotor.class, "left_back");
        right_front_motor = hardwareMap.get(DcMotor.class, "right_front");
        right_back_motor = hardwareMap.get(DcMotor.class, "right_back");

        //telling motors to stop with a break
        //~drive~
        left_front_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake_motor = hardwareMap.get(DcMotor.class, "intake");
        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_motor.setDirection(DcMotor.Direction.REVERSE);
        intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right_intake_servo = hardwareMap.get(CRServo.class, "rightwristservo");
        right_intake_servo.setDirection(CRServo.Direction.REVERSE);
        left_intake_servo = hardwareMap.get(CRServo.class, "leftwristservo");
        drone_servo = hardwareMap.get(Servo.class, "drone");
        wristServo_servo = hardwareMap.servo.get("wristservo");
//droneLauncher_servo.setDirection(Servo.Direction.REVERSE);
        rightLinearSlide_motor = hardwareMap.get(DcMotor.class, "rightlinearslide");
        leftLinearSlide_motor = hardwareMap.get(DcMotor.class, "leftlinearslide");
        rightLinearSlide_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLinearSlide_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLinearSlide_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLinearSlide_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void reset_encoders() {
        rightLinearSlide_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLinearSlide_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void LinearSlidesMoveToPosition(int degrees) {
        reset_encoders();
        rightLinearSlide_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLinearSlide_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linear_slides_tic_value = (int) Math.round(degrees * LINEAR_SLIDES_DEGREES_PER_TIC);
        rightLinearSlide_motor.setTargetPosition(linear_slides_tic_value);
        leftLinearSlide_motor.setTargetPosition(linear_slides_tic_value);
        rightLinearSlide_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLinearSlide_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLinearSlide_motor.setPower(linearSlidePowerValue);
        leftLinearSlide_motor.setPower(linearSlidePowerValue);
    }

    public void start() {
        reset_encoders();
    }

    public void Drivetrain() {
        if (cur_gamepad1rightbumper && !prev_gamepad1rightbumper) {
            speed = DRIVE_MIN_SPEED;
        } else {
            speed = DRIVE_MAX_SPEED;
        }


        //has max power of 1 so multiplying it by 0.25 will change its max to 0.25, moving it 4x slower
        left_front_motor.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x) * speed);
        left_back_motor.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x) * speed);
        right_front_motor.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x) * speed);
        right_back_motor.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x) * speed);

    }
    public void booleanKeys() {
        prev_gamepad1rightbumper = cur_gamepad1rightbumper;
        cur_gamepad1rightbumper = gamepad1.right_bumper;
        prev_gamepad2b = cur_gamepad2b;
        cur_gamepad2b = gamepad2.b;
        prev_gamepad2rb = cur_gamepad2rb;
        cur_gamepad2rb = gamepad2.right_bumper;
        prev_gamepad2lb = cur_gamepad2lb;
        cur_gamepad2lb = gamepad2.left_bumper;
        prev_gamepad2dpad_l = cur_gamepad2dpad_l;
        cur_gamepad2dpad_l = gamepad2.dpad_left;
        prev_gamepad2dpad_r = cur_gamepad2dpad_r;
        cur_gamepad2dpad_r = gamepad2.dpad_right;
    }

    public void intakeAndOutake() {
        if (gamepad2.right_trigger > 0.08) {
            right_intake_servo.setPower(0.995);
            left_intake_servo.setPower(0.995);
            intake_motor.setPower(0.995);
        }
        else if (cur_gamepad2b && prev_gamepad2b) {
            right_intake_servo.setPower(-0.995);
            left_intake_servo.setPower(-0.995);
            intake_motor.setPower(-0.995);
        }
        else {
            right_intake_servo.setPower(0);
            left_intake_servo.setPower(0);
            intake_motor.setPower(0);
        }

        if(cur_gamepad2dpad_r && prev_gamepad2dpad_r){
            right_intake_servo.setPower(-0.995);
        }
        else{
            right_intake_servo.setPower(0);
        }

        if(cur_gamepad2dpad_l && prev_gamepad2dpad_l){
            left_intake_servo.setPower(-0.995);
        }
        else{
            left_intake_servo.setPower(0);
        }

        if (cur_gamepad2dpad_r && !prev_gamepad2dpad_r) {
            drone_servo.setPosition(1);
            //launcher_servo.setPosition(0.5);
        }
        else{
            drone_servo.setPosition(0);
        }


        if (cur_gamepad2lb && !prev_gamepad2lb) {// preset 2 - intake position
            wristServo_servo.setPosition(WristPresetTwo);
            LinearSlidesMoveToPosition(LinearSlidePresetTwo);
        }
        else if (cur_gamepad2rb && !prev_gamepad2rb) {// preset 1 - scoring position
            LinearSlidesMoveToPosition(LinearSlidePresetOne);
            wristServo_servo.setPosition(WristPresetOne);
        }
        else if (Math.abs(gamepad2.left_stick_y) > 0.02) {// manual control
            //Manual linear slide control
            rightLinearSlide_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftLinearSlide_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (gamepad2.left_stick_y > 0.02) {

                rightLinearSlide_motor.setPower(0.5);
                leftLinearSlide_motor.setPower(0.5);
            }
            else if (gamepad2.left_stick_y < 0.02) {
                rightLinearSlide_motor.setPower(-0.5);
                leftLinearSlide_motor.setPower(-0.5);
            }

        }
        else if (Math.abs(gamepad2.right_stick_y) > 0.02){
            //Manual wrist control
            if (gamepad2.right_stick_y > 0.02) {
                wristServo_servo.setPosition(wristServo_servo.getPosition() + 0.2);
            }
            else if (gamepad2.right_stick_y < 0.02) {
                wristServo_servo.setPosition(wristServo_servo.getPosition() - 0.2);
            }
        }
        else{
            rightLinearSlide_motor.setPower(0);
            leftLinearSlide_motor.setPower(0);
        }

        //outtake
        if (gamepad2.left_trigger > 0.08) {
            right_intake_servo.setPower(-0.995);
            left_intake_servo.setPower(-0.995);
        }
        else {
            right_intake_servo.setPower(0);
            left_intake_servo.setPower(0);
        }
    }

    @Override
    public void loop() {
        booleanKeys();
        Drivetrain();
        intakeAndOutake();
    }
}