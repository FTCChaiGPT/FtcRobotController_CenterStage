package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name = "Dec2Drive", group = "iterativeOpMode")
public class Dec2DriverEnhancedPeriod extends OpMode {
    private DcMotor intake_motor;//intake
    private Servo pusher_servo;//intake
    private Servo gate_servo;//intake
    private Servo scooper_servo;//intake
    private DcMotor hangOrientor_motor;//hang
    private DcMotor leftHang_motor;//hang
    private Servo launcher_servo;//drone
    private DcMotor left_front_motor;//drive
    private DcMotor left_back_motor;//drive
    private DcMotor right_front_motor;//drive
    private DcMotor right_back_motor;//drive
    private DcMotor arm_motor;//
    private Servo wrist_servo;//
    private Servo claw_servo;//
    private Servo telescoping_servo;
    private final double SCOOPER_DROP = 220.0 / 270.0; // normalized for 220 degrees
    private final double SCOOPER_RESET = 130.0 / 270.0; // normalized for 130 degrees
    private ElapsedTime runtime = new ElapsedTime(); //timer, used to remove sleeps
    private double checkEachInstance = 0;//keeps track of each iteration of outtake
    private double outtakeHold = 0;//finishes an outtake-run, instead of stopping mid-way when you release the button
    private double[] outtakeValues = {0.0, 0.995, 0.015, 0.565};//{gate, pusher, pusher, gate}
    //                              {gate, pusher, pusher, gate};
    private double WAIT_TIME = 85;// in milliseconds
    private double speed = 1;//drive speed
    private final double CLOSED_POSITION = 0.5;
    private final double OPEN_POSITION = 0.02;
    private boolean isAtClosedPosition = false;
    private int dpad_left_stages = 2;
    private boolean atScooperResetPosition = true;
    private static final double COUNTS_PER_MOTOR_REV = 750; //Number of encoder counts per motor revolution (750)
    private static final double ARM_LENGTH = 3.75;
    private static final double GEAR_RATIO = 1.0;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * GEAR_RATIO) / (ARM_LENGTH * Math.PI);
    private static final double ROBOT_WIDTH_INCHES = 18.9; // The distance between the wheels on opposite sides (diagonal)
    private int armStagesIterator = 0;
    private int armHold = 0;
    private int[] armStages = {0, 160, 225, 180};//DcMotor with encoder
    private double[] wristStages = {0.02, 0.5, 0.833, 0.995};//Angular Servo
    @Override
    public void init() {
        //calling claw and wrist servos
        claw_servo = hardwareMap.servo.get("claw_servo");
        telescoping_servo = hardwareMap.servo.get("telescoping_servo");
        wrist_servo = hardwareMap.servo.get("wrist_servo");
        arm_motor = hardwareMap.get(DcMotor.class, "arm_motor");
        //~arm~
        arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wrist_servo.setDirection(Servo.Direction.FORWARD);

        launcher_servo = hardwareMap.servo.get("launcher");
        launcher_servo.setDirection(Servo.Direction.REVERSE);

        hangOrientor_motor = hardwareMap.get(DcMotor.class, "Orient");
        leftHang_motor = hardwareMap.get(DcMotor.class, "LeftHang");

        intake_motor = hardwareMap.get(DcMotor.class, "intake");
        pusher_servo = hardwareMap.servo.get("pusher");
        gate_servo = hardwareMap.servo.get("gate");
        scooper_servo = hardwareMap.servo.get("front");
        intake_motor.setDirection(DcMotor.Direction.FORWARD);
        pusher_servo.setDirection(Servo.Direction.FORWARD);

        left_front_motor = hardwareMap.get(DcMotor.class, "left_front");
        left_back_motor = hardwareMap.get(DcMotor.class, "left_back");
        right_front_motor = hardwareMap.get(DcMotor.class, "right_front");
        right_back_motor = hardwareMap.get(DcMotor.class, "right_back");
        right_front_motor.setDirection(DcMotor.Direction.REVERSE);
        right_back_motor.setDirection(DcMotor.Direction.REVERSE);

        //telling motors to stop with a break
        //~drive~
        left_front_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //~hang~
        hangOrientor_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftHang_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        runtime.reset(); //resets the ElapsedTime timer
    }

    public void armUp(int degrees, double power) {
        resetEncoders();

        double radians = Math.toRadians(degrees);
        double wheelDistance = radians * (ROBOT_WIDTH_INCHES / 2);

        int encoderCounts = (int) (wheelDistance * COUNTS_PER_INCH);

        arm_motor.setTargetPosition(encoderCounts);
        arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setMotorPower(power);
        stopMotor();
    }

    private void setMotorPower(double x) {
        arm_motor.setPower(x);
    }

    public void armDown(int degrees, double power) {
        resetEncoders();

        double radians = Math.toRadians(degrees);
        double wheelDistance = radians * (ROBOT_WIDTH_INCHES / 2);

        int encoderCounts = (int) (wheelDistance * COUNTS_PER_INCH);

        arm_motor.setTargetPosition(-encoderCounts);
        arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setMotorPower(power);
        stopMotor();
    }

    public void stopMotor() {
        setMotorPower(0);
        resetEncoders();
    }

    private void resetEncoders() {
        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void Drivetrain() {
        //has max power of 1 so multiplying it by 0.25 will change its max to 0.25, moving it 4x slower
        left_front_motor.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x) * speed);
        left_back_motor.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x) * speed);
        right_front_motor.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x) * speed);
        right_back_motor.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x) * speed);

        if (gamepad1.right_bumper) { //once clicked, changes speed to alternate value, toggles between 0.25 and 1
            if (speed == 1) {
                speed = 0.25;
            } else {
                speed = 1;
            }
        }
    }

    public void ArmFunction() {

        if (gamepad2.b || armHold == 1) {
            armHold = 1;
        }

        if (armHold == 1) {
            armStagesIterator++;
            armUp(armStages[armStagesIterator], 1);
            wrist_servo.setPosition(wristStages[armStagesIterator]);

            if (armStagesIterator == 4) {
                armStagesIterator = 0;
                armHold = 0;
            }
        }

        if (gamepad2.a) {//claw opens and closes when a pressed.
            if (isAtClosedPosition == true) {
                isAtClosedPosition = false;
                claw_servo.setPosition(CLOSED_POSITION);
            } else {
                isAtClosedPosition = true;
                claw_servo.setPosition(OPEN_POSITION);
            }
        }

        if (gamepad2.x) {
            telescoping_servo.setPosition(1);
        } else if (gamepad2.x) {
            telescoping_servo.setPosition(-1);
        }
    }

    public void Intake() {
        if (gamepad2.right_bumper) {//forward intake
            intake_motor.setPower(-1);
            telemetry.addLine("Intake Is Spinning Forward.");
        } else if (gamepad2.left_trigger > 0.08) {//backward intake
            intake_motor.setPower(0.4);
            telemetry.addLine("Intake Is Spinning Backwards.");
        } else {//stop intake
            intake_motor.setPower(0);
        }

        if (gamepad2.left_bumper || outtakeHold == 1) { //outtake
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

        if (gamepad2.right_trigger > 0.08) {//brings scooper down
            atScooperResetPosition = false;
        }
        else {
            atScooperResetPosition = true;
        }

        if (atScooperResetPosition == false) {
            scooper_servo.setPosition(SCOOPER_RESET);
        }
        else {
            scooper_servo.setPosition(SCOOPER_DROP);
        }
    }

    public void HangPlusDrone() {
        if (gamepad2.dpad_up) {//moves hang up by 45 degrees
            hangOrientor_motor.setPower(1);
            sleep(270);
            hangOrientor_motor.setPower(0);
        }
        if (gamepad2.dpad_down) {//brings hang down
            hangOrientor_motor.setPower(-1);
            sleep(525);
            hangOrientor_motor.setPower(0);
        }

        if (gamepad2.dpad_right) {
            //extends hang when both joysticks are up
            leftHang_motor.setPower(1);
        } else if (gamepad2.dpad_left && dpad_left_stages == 2) {
            leftHang_motor.setPower(-1);
        } else {
            //if nothing hang-actuator-related is happening, stop all hang movement
            leftHang_motor.setPower(0);
        }
        if (gamepad2.dpad_left && dpad_left_stages == 1) {
            launcher_servo.setPosition(0); //launch drone is both triggers are pressed.
            dpad_left_stages = 2;
        }
    }

    public void start() {
        gate_servo.setPosition(outtakeValues[3]);
        pusher_servo.setPosition(outtakeValues[2]);
        scooper_servo.setPosition(SCOOPER_DROP);
        wrist_servo.setPosition(0.666);

        resetEncoders();
    }


    @Override
    public void loop() {
        Drivetrain();
        ArmFunction();
        Intake();
        HangPlusDrone();

        //update telemetry
        telemetry.update();
    }
}

