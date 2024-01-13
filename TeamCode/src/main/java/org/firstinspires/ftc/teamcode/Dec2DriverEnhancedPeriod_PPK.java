package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.setCurrentTimeMillis;
import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Dec2DriverEnhancedPeriod_PPK", group = "iterativeOpMode")
public class Dec2DriverEnhancedPeriod_PPK extends OpMode {
    private DcMotor left_front_motor;//drive
    private DcMotor left_back_motor;//drive
    private DcMotor right_front_motor;//drive
    private DcMotor right_back_motor;//drive
    private DcMotor intake_motor;//intake
    private DcMotor leftHang_motor;//hang
    private DcMotor hangOrientor_motor;//hang
    private DcMotor arm_motor;//

    private Servo pusher_servo;//intake
    private Servo gate_servo;//intake
    private Servo scooper_servo;//intake
    private Servo launcher_servo;//drone
    private Servo wrist_servo;//
    private Servo claw_servo;//
    private Servo telescope_servo;

    // arm motor calculations for degree changes
    private static final double COUNTS_PER_MOTOR_REV_ARM = 5281;
    private static final double ARM_DEGREES_PER_TIC = COUNTS_PER_MOTOR_REV_ARM / 360;
    private int arm_degree_position ;
    private int arm_tic_value;


    // Set Motor power values.
    private double motorPowerValue = 0.5;
    private double intakePowerValue = 0.75;
    private double armPowerValue = 0.6;
    private double speed = 1;//drive speed

    // Set SERVO values.

    private static final double WRIST_BACKDROP_DROP = 0.55;
    private static final double WRIST_RESET_TO_HANG = 0.995;
    private static final double WRIST_PICKUP = 0.43;
    private static final double CLAW_OPEN = 0.2;
    private static final double CLAW_CLOSE = 0.75;
    private static final double PUSHER_RESET = 0.015;
    private static final double GATE_RESET = 0.565;
    private static final double TELESCOPE_SHORT = 0.02;
    private static final double TELESCOPE_EXTEND = 0.995;
    private static final double SCOOPER_MAX_POSITION = 220.0/270.0; // normalized for 270 degrees
    private static final double SCOOPER_MIN_POSITION = 130.0/270.0; // normalized for 270 degrees

    private static final double DRIVE_MAX_SPEED =1.0;
    private static final double DRIVE_MIN_SPEED =0.25;

    private static final double REVERSE_INTAKE_SPEED = -0.65;
    private static final double INTAKE_SPEED = 1.0;

    private boolean prev_gamepad2a = false;
    private boolean prev_gamepad2b = false;
    private boolean prev_gamepad2x = false;
    private boolean prev_gamepad2y = false;
    private boolean prev_gamepad2rightBumper = false;
    private boolean prev_gamepad2leftBumper = false;
    private boolean prev_gamepad1y = false;
    private boolean prev_gamepad1a = false;
    private boolean cur_gamepad2a = false;
    private boolean cur_gamepad2b = false;
    private boolean cur_gamepad2x = false;
    private boolean cur_gamepad2y = false;
    private boolean cur_gamepad1y = false;
    private boolean cur_gamepad1a = false;
    private boolean cur_gamepad2rightBumper = false;
    private boolean cur_gamepad2leftBumper = false;
    private boolean cur_gamepad1x = false;
    private boolean prev_gamepad1x = false;


    private ElapsedTime runtime = new ElapsedTime(); //timer, used to remove sleeps
    private double checkEachInstance = 0;//keeps track of each iteration of outtake
    private double outtakeHold = 0;//finishes an outtake-run, instead of stopping mid-way when you release the button
    private double[] outtakeValues = {0.0, 0.995, 0.015, 0.565};//{gate, pusher, pusher, gate}
    //                              {gate, pusher, pusher, gate};
    private double WAIT_TIME = 85;// in milliseconds

    private boolean isClawOpenPosition;
    private boolean isTelescopeShort;
    private boolean isWristDropPosition;
    private boolean isArmPickUpPosition;
    private boolean isWristPickUpPosition;
    private boolean isScooperDown = true;
    private boolean isArmDropPosition;
    private boolean readytoResetEncoders;
    private int dpad_left_stages = 2;
    int maxArmPosition = 210;
    int maxArmPosition2 = 220;
    int targetArmPosition = 165;
    double wristServoPosition = 0.43;
    boolean speedchange;


    @Override
    public void init() {

        left_front_motor = hardwareMap.get(DcMotor.class, "left_front");
        left_back_motor = hardwareMap.get(DcMotor.class, "left_back");
        right_front_motor = hardwareMap.get(DcMotor.class, "right_front");
        right_back_motor = hardwareMap.get(DcMotor.class, "right_back");
        hangOrientor_motor = hardwareMap.get(DcMotor.class, "Orient");
        leftHang_motor = hardwareMap.get(DcMotor.class, "LeftHang");
        intake_motor = hardwareMap.get(DcMotor.class, "intake");

        //telling motors to stop with a break
        //~drive~
        left_front_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //~hang~
        hangOrientor_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftHang_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake_motor.setDirection(DcMotor.Direction.REVERSE);
        right_front_motor.setDirection(DcMotor.Direction.REVERSE);
        right_back_motor.setDirection(DcMotor.Direction.REVERSE);


        //servos
        launcher_servo = hardwareMap.servo.get("launcher");
        pusher_servo = hardwareMap.servo.get("pusher");
        gate_servo = hardwareMap.servo.get("gate");
        scooper_servo = hardwareMap.servo.get("front");

        pusher_servo.setDirection(Servo.Direction.FORWARD);
        launcher_servo.setDirection(Servo.Direction.REVERSE);


        //Dec2 changes
        // New Motors
        arm_motor = hardwareMap.get(DcMotor.class, "arm_motor");
        arm_motor.setDirection(DcMotor.Direction.REVERSE);
        arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // set its current position to 0 - so this can be used for all times.

        //new servos
        claw_servo = hardwareMap.get(Servo.class, "claw_servo");
        telescope_servo = hardwareMap.get(Servo.class, "telescoping_servo");
        wrist_servo = hardwareMap.get(Servo.class, "wrist_servo");

        wrist_servo.setDirection(Servo.Direction.FORWARD);

        runtime.reset(); //resets the ElapsedTime timer

        speedchange = false;

    }

    public void start() {
        //-update
        arm_motor.setTargetPosition(0);
        arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        claw_servo.setPosition(CLAW_CLOSE);
        isClawOpenPosition = false;

        wrist_servo.setPosition(WRIST_PICKUP);
        gate_servo.setPosition(GATE_RESET);
        pusher_servo.setPosition(PUSHER_RESET);
        scooper_servo.setPosition(SCOOPER_MAX_POSITION);
        telescope_servo.setPosition(TELESCOPE_SHORT);

        isScooperDown = false;
        isTelescopeShort = false;
        isArmPickUpPosition = false;
        isWristDropPosition = false;
        isWristPickUpPosition= false;
        isArmDropPosition = false;
        readytoResetEncoders = false;
    }

    public void Gamepad_BooleanKeys(){
        //save all the values from the previous run
        prev_gamepad2a = cur_gamepad2a;
        prev_gamepad2b = cur_gamepad2b;
        prev_gamepad2x = cur_gamepad2x;
        prev_gamepad2y = cur_gamepad2y;
        prev_gamepad2rightBumper = cur_gamepad2rightBumper;
        prev_gamepad2leftBumper = cur_gamepad2leftBumper;
        prev_gamepad1x = cur_gamepad1x;
        prev_gamepad1y = cur_gamepad1y;
        prev_gamepad1a = cur_gamepad1a;
        //save all the values from the current run
        cur_gamepad2a = gamepad2.a;
        cur_gamepad2b = gamepad2.b;
        cur_gamepad2x = gamepad2.x;
        cur_gamepad2y = gamepad2.y;
        cur_gamepad2rightBumper = gamepad2.right_bumper;
        cur_gamepad2leftBumper = gamepad2.left_bumper;
        cur_gamepad1x = gamepad1.x;
        cur_gamepad1y = gamepad1.y;
        cur_gamepad1a = gamepad1.a;
    }

    public  void Elements() {
        Gamepad_BooleanKeys();

        // Gamepad2:
        //  ----------------- Boolean Controls ------------
        //   GAMEPAD 2
        //   right Bumper - Intake          - You hold to operate
        //   left Bumper -Pusher & Gate     - you press once - it does and resets
        //   A - Preset- Arm Pick
        //   B - Preset- Arm Drop
        //   Y - Claw Open & Close          - toggle between open and close
        //   X - Telescope Extend           - toggle between short and extend
        //   Dpad UP - Orient Up
        //   Dpad Down - Orient Down
        //   Dpad Right - Extend Hang
        //   Dpad Left - Pull up Hang
        //
        //   ----------------- Analog Controls ------------
        //   left_stick x/y -Arm Angle
        //   right_stick x/y - Wrist Angle
        //   left trigger -
        //   right trigger -
        //
        //   GAMEPAD 1
        //
        //   Boolean - Wrist Position, Drone,
        //   Variable - Orient Angle, Hang Motor
        //   Driver GamePad:
        //      Y scooper up/down
        //      A Reverse Intake
        //      X Encoder Resets


        // Intake Pixel Code
        // Gamepad2.Right Bumper
        if (gamepad2.right_bumper) {
            intake_motor.setPower(INTAKE_SPEED);   // Intake Pixel
            telemetry.addLine("Intake Pixel is On.");
        } else {
            intake_motor.setPower(0);  //Intake Stop
        }

        // Pusher & Gate Operation
        // Gamepad2.Left Bumper
        if ((cur_gamepad2leftBumper && !prev_gamepad2leftBumper) || outtakeHold == 1) { //outtake
            outtakeHold = 1;
            //checks each iteration and applies a break in between (WAIT_TIME)
            if (checkEachInstance == 0 && runtime.milliseconds() > WAIT_TIME) {
                gate_servo.setPosition(outtakeValues[0]);
                checkEachInstance++;
                runtime.reset();
            } //checks for 8 iterations and moves an eighth of 0.995 (outtakeValues[1])
            else if (checkEachInstance < 8 && runtime.milliseconds() > WAIT_TIME) {
                pusher_servo.setPosition((outtakeValues[1] * 0.125) + pusher_servo.getPosition());
                checkEachInstance++;
                runtime.reset();
            } //returns pusher back to rest position (outtakeValues[0])
            else if (checkEachInstance == 8 && runtime.milliseconds() > (WAIT_TIME + 15)) {
                pusher_servo.setPosition(outtakeValues[2]);
                checkEachInstance++;
                runtime.reset();
            } //returns gate back to rest position (outtakeValues[0])
            else if (checkEachInstance == 9 && runtime.milliseconds() > WAIT_TIME) {
                gate_servo.setPosition(outtakeValues[3]);
                checkEachInstance++;
                runtime.reset();
            }
            if (checkEachInstance > 9) {// "breaks" out of this cycle and resets parameters
                checkEachInstance++;
                if (checkEachInstance == 12) {//gets denied once to offer a break which preventing bumpy driving
                    telemetry.addData("Outtake action complete! ", checkEachInstance);
                    checkEachInstance = 0;
                    outtakeHold = 0;
                    runtime.reset();
                    telemetry.addData("gate", gate_servo.getPosition());
                    telemetry.addData("pusher", pusher_servo.getPosition());
                    telemetry.update();
                }
            }
        }

        // GamePad2.Y - Claw Open & Close
        if (cur_gamepad2y && !prev_gamepad2y) {
            isClawOpenPosition = !isClawOpenPosition;
        }
        if (isClawOpenPosition) {
            claw_servo.setPosition(CLAW_CLOSE);
            telemetry.addLine("Claw Closed");
            telemetry.update();
        } else {
            claw_servo.setPosition(CLAW_OPEN);
            telemetry.addLine("Claw Opened");
            telemetry.update();
        }

        if (cur_gamepad2x && !prev_gamepad2x) {
            isTelescopeShort = !isTelescopeShort;
        }
        if (isTelescopeShort) {
            telescope_servo.setPosition(TELESCOPE_EXTEND);
            telemetry.addLine("Telescope Extended");
            telemetry.update();
        } else {
            telescope_servo.setPosition(TELESCOPE_SHORT);
            telemetry.addLine("Telescope Short");
            telemetry.update();
        }

//        if(cur_gamepad2a && !prev_gamepad2a){
//            isArmPickUpPosition = true;
//            // ArmPickFunction()
//            // Set the arm_motor to the target position (210 degrees)
//        } else if (cur_gamepad2b && !prev_gamepad2b) {
//            // ArmDropFunction()
//            isArmDropPosition = true;
//        } else if (Math.abs(gamepad1.left_stick_y) > 0.2 ) {
//            // Limit the range of motion for arm_motor using left stick
//            double leftStickY = -gamepad1.left_stick_y; // Reverse Y axis to match motor direction
//
//            int currentArmPosition = arm_motor.getCurrentPosition();
//            int newArmPosition = currentArmPosition + (int) (leftStickY * 10); // Adjust the multiplier as needed
//
//            // Ensure the newArmPosition stays within the maximum allowed position
//            newArmPosition = Math.min(maxArmPosition2, Math.max(0, newArmPosition));
//
//            // Control arm_motor using the newArmPosition
//            arm_motor.setTargetPosition(newArmPosition);
//            arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            arm_motor.setPower(armPowerValue); // Adjust power as needed
//
//        } else if(Math.abs(gamepad1.right_stick_y) > 0.2){
//            double rightStickY = -gamepad1.right_stick_y; // Reverse Y axis if needed
//            // Control wrist_servo using right stick
//            wristServoPosition += rightStickY * 0.01; // Adjust the multiplier to change servo speed
//
//            // Ensure wrist_servo position stays within the desired range (0.43 to 0.55)
//            wristServoPosition = Math.min(WRIST_BACKDROP_DROP, Math.max(WRIST_PICKUP, wristServoPosition));
//            wrist_servo.setPosition(wristServoPosition);
//        }
//
//        if(isArmPickUpPosition){
//            ArmPickupPosition();
//            isArmPickUpPosition = false;
//        }
//
//        if(isArmDropPosition){
//            ArmDropPosition();
//            isArmDropPosition = false;
//        }
//
//

        // GamePad2.Left Stick Y  - Manual Arm Operation
        if(gamepad2.left_stick_y < 0.2){
            arm_motor.setPower(0.75 * gamepad2.left_stick_y);
            arm_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else if (gamepad2.left_stick_y > -0.2) {
            arm_motor.setPower(0.75 * gamepad2.left_stick_y);
            arm_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            arm_motor.setPower(0);
        }

        // GamePad2.Right Stick Y  - Manual Wrist Operation
        if(gamepad2.right_stick_y >0.2){
            wrist_servo.setPosition(0.55);
        } else if (gamepad2.right_stick_y <-0.2) {
            wrist_servo.setPosition(0.43);
        }

        // GamePad1.X  - Do Reset of Encoders
        if(cur_gamepad1x && !prev_gamepad1x){
            reset_encoders();
        }

        // GamePad1.Y  - Reverse Intake
        if(gamepad1.y) {
            intake_motor.setPower(REVERSE_INTAKE_SPEED);
        }

        // GamePad1.A  - Scooper Lift & Drop
        if(cur_gamepad1a && ! prev_gamepad1a) {
            isScooperDown = !isScooperDown;
        }

        if (isScooperDown){
            scooper_servo.setPosition(SCOOPER_MIN_POSITION);
        } else {
            scooper_servo.setPosition(SCOOPER_MAX_POSITION);
        }

    }

    public void reset_encoders(){
        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //hangOrientor_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftHang_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void ArmDropPosition(){
        arm_motor.setTargetPosition(targetArmPosition); // Set the maximum allowed position
        arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm_motor.setPower(armPowerValue); // Adjust power as needed
        // Set the wrist_servo to the preset position (WRIST_PICKUP)
        wrist_servo.setPosition(WRIST_BACKDROP_DROP);

    }


    public void ArmPickupPosition(){
        arm_motor.setTargetPosition(maxArmPosition); // Set the maximum allowed position
        arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm_motor.setPower(armPowerValue); // Adjust power as needed
        // Set the wrist_servo to the preset position (WRIST_PICKUP)
        wrist_servo.setPosition(WRIST_PICKUP);

    }

    public void HangPlusDrone() {

        // Gamepad2  dpad_up  - Get Orient to 45 / 90 degree & ready to launch
        // Gamepad2  dpad_down - Get Orient back to reset point
        // Gamepad2  dpad_right  - Left hang extend out
        // Gamepad2  dpad_left  - Left hang pull it back
        // Gamepad1  dpad_up  - Drone Launch

        if (gamepad2.dpad_up) {//moves hang up by 45 degrees
            hangOrientor_motor.setPower(1);
            sleep(270);
            hangOrientor_motor.setPower(0);
        } else {
            hangOrientor_motor.setPower(0);
        }
        if (gamepad2.dpad_down) {//brings hang down
            hangOrientor_motor.setPower(-1);
            sleep(525);
            hangOrientor_motor.setPower(0);
        } else {
            hangOrientor_motor.setPower(0);
        }

        if (gamepad2.dpad_right) {
            leftHang_motor.setPower(1);
        } else if (gamepad2.dpad_left) {
            leftHang_motor.setPower(-1);
        } else {
            leftHang_motor.setPower(0);
        }

    }

    public void Drivetrain() {


        if(gamepad1.right_bumper){
            speedchange = !speedchange;
        }

        if(speedchange){
            speed = DRIVE_MIN_SPEED;
        } else{
            speed = DRIVE_MAX_SPEED;
        }

//        if (gamepad1.right_bumper) { //once clicked, changes speed to alternate value, toggles between 0.25 and 1
//            if (speed == DRIVE_MAX_SPEED) {
//                speed = DRIVE_MIN_SPEED;
//            } else {
//                speed = DRIVE_MAX_SPEED;
//            }
//        }

        //has max power of 1 so multiplying it by 0.25 will change its max to 0.25, moving it 4x slower
        left_front_motor.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x) * speed);
        left_back_motor.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x) * speed);
        right_front_motor.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x) * speed);
        right_back_motor.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x) * speed);

    }

    @Override
    public void loop() {

        Drivetrain();
        Elements();
        HangPlusDrone();
        if (gamepad1.right_trigger>0.5) {
            launcher_servo.setPosition(1);
        }

        //update telemetry
        telemetry.update();
    }
}