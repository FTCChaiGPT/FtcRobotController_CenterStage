package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Bot3DriverEnhancedPeriod", group = "iterativeOpMode")
public class Bot3DriverEnhancedPeriod extends OpMode {
    //initialization
    private DcMotor left_front_motor;//drive
    private DcMotor left_back_motor;//drive
    private DcMotor right_front_motor;//drive
    private DcMotor right_back_motor;//drive
    private CRServo drone_servo;
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
    private boolean cur_gamepad2dpad_up;
    private boolean prev_gamepad2dpad_up;
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
    private int WristPresetOne =  1; //scoring position
    //private int WristPresetOne =  50/270;
    //private int WristPresetOne =  30/270;
    private int WristPresetTwo = 0; //intake position
    @Override
    public void init() {
        left_front_motor = hardwareMap.get(DcMotor.class, "left_front");
        left_back_motor = hardwareMap.get(DcMotor.class, "left_back");
        right_front_motor = hardwareMap.get(DcMotor.class, "right_front");
        right_back_motor = hardwareMap.get(DcMotor.class, "right_back");

        //telling motors to stop with a break
        //~drive~ turning reverse strafing reverse
        left_front_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_motor.setDirection(DcMotor.Direction.REVERSE);
        right_back_motor.setDirection(DcMotor.Direction.REVERSE);


        intake_motor = hardwareMap.get(DcMotor.class, "intake");
        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_motor.setDirection(DcMotor.Direction.REVERSE);
        intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right_intake_servo = hardwareMap.get(CRServo.class, "rightwristservo");
        right_intake_servo.setDirection(CRServo.Direction.REVERSE);
        left_intake_servo = hardwareMap.get(CRServo.class, "leftwristservo");
        drone_servo = hardwareMap.get(CRServo.class, "drone");
        wristServo_servo = hardwareMap.servo.get("wristservo");
        drone_servo.setDirection(CRServo.Direction.REVERSE);
        rightLinearSlide_motor = hardwareMap.get(DcMotor.class, "rightlinearslide");
        leftLinearSlide_motor = hardwareMap.get(DcMotor.class, "leftlinearslide");
        rightLinearSlide_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLinearSlide_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLinearSlide_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLinearSlide_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLinearSlide_motor.setDirection(DcMotor.Direction.REVERSE);
        leftLinearSlide_motor.setDirection(DcMotor.Direction.FORWARD);
        wristServo_servo.setPosition(WristPresetTwo);
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


        left_front_motor.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x) * speed);
        left_back_motor.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x) * speed);
        right_front_motor.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x) * speed);
        right_back_motor.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x) * speed);

// Calculate individual motor powers
        //   double forwardBackward = gamepad1.left_stick_y * speed;
        //   double strafe = gamepad1.right_stick_x * speed;
        //   double turn = -gamepad1.right_stick_y * speed; // Invert to match typical joystick behavior

// Calculate motor powers for mecanum drive
        //   double leftFrontPower = forwardBackward + strafe + turn;
        //   double leftBackPower = forwardBackward - strafe + turn;
        //   double rightFrontPower = forwardBackward - strafe - turn;
        //    double rightBackPower = forwardBackward + strafe - turn;

// Set motor powers
        //   left_front_motor.setPower(leftFrontPower);
        //   left_back_motor.setPower(leftBackPower);
        //   right_front_motor.setPower(rightFrontPower);
        //  right_back_motor.setPower(rightBackPower);

        //has max power of 1 so multiplying it by 0.25 will change its max to 0.25, moving it 4x slower
        //  left_front_motor.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x) * speed);
        // left_back_motor.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x) * speed);
        // right_front_motor.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x) * speed);
        // right_back_motor.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x) * speed);



    }
    public void booleanKeys() {
        prev_gamepad1rightbumper = cur_gamepad1rightbumper;//drive speed
        cur_gamepad1rightbumper = gamepad1.right_bumper;//drive speed
        prev_gamepad2b = cur_gamepad2b;//outtake
        cur_gamepad2b = gamepad2.b;//outtake
        prev_gamepad2rb = cur_gamepad2rb; //preset 1-scoring
        cur_gamepad2rb = gamepad2.right_bumper; //preset 1-scoring
        prev_gamepad2lb = cur_gamepad2lb;//preset 2-intake
        cur_gamepad2lb = gamepad2.left_bumper;//preset 2-intake
        prev_gamepad2dpad_l = cur_gamepad2dpad_l;//left intake
        cur_gamepad2dpad_l = gamepad2.dpad_left;//left intake
        prev_gamepad2dpad_r = cur_gamepad2dpad_r;//right intake
        cur_gamepad2dpad_r = gamepad2.dpad_right;//right intake
        prev_gamepad2dpad_up = cur_gamepad2dpad_up;//drone
        cur_gamepad2dpad_up = gamepad2.dpad_up;//drone
        //joystick right- wrist
        //joystick left- slides
        //right trigger- intake
        //left trigger- reverse intake
    }


    public void intakeAndOuttake() {
        if (gamepad2.right_trigger > 0.08) {//intake
            right_intake_servo.setPower(0.995);
            left_intake_servo.setPower(0.995);
            intake_motor.setPower(0.995);
            wristServo_servo.setPosition(WristPresetTwo);
        }
        else if (cur_gamepad2b && prev_gamepad2b) {//reverse-intake
            intake_motor.setPower(-0.995);
        }
        else {
            right_intake_servo.setPower(0);
            left_intake_servo.setPower(0);
            intake_motor.setPower(0);
        }

        if(cur_gamepad2dpad_r && prev_gamepad2dpad_r){//right
            right_intake_servo.setPower(-0.995);
        }
        else{
            right_intake_servo.setPower(0);
        }

        if(cur_gamepad2dpad_l && prev_gamepad2dpad_l){//left
            left_intake_servo.setPower(-0.995);
        }
        else{
            left_intake_servo.setPower(0);
        }
//drone
        if (cur_gamepad2dpad_up && prev_gamepad2dpad_up) {
            drone_servo.setPower(0.5);
        }
        else {
            drone_servo.setPower(0.0);
        }




        if (cur_gamepad2lb && !prev_gamepad2lb) {// preset 2 - intake position
            wristServo_servo.setPosition(WristPresetTwo);
        }
        else if (cur_gamepad2rb && !prev_gamepad2rb) {// preset 1 - scoring position
            wristServo_servo.setPosition(WristPresetOne);
        }
        else if (Math.abs(gamepad2.left_stick_y) > 0.02) {// manual control
            //Manual linear slide control
            rightLinearSlide_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftLinearSlide_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (gamepad2.left_stick_y > 0.02) {
                rightLinearSlide_motor.setPower(0.65);
                leftLinearSlide_motor.setPower(0.65);
            }
            else if (gamepad2.left_stick_y < 0.02) {
                rightLinearSlide_motor.setPower(-0.65);
                leftLinearSlide_motor.setPower(-0.65);
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
        intakeAndOuttake();
    }
}