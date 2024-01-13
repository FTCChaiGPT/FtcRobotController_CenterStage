package org.firstinspires.ftc.teamcode;


import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "DrivetrainFinal", group = "iterativeOpMode")
public class DriverEnhancedPeriod extends OpMode {
    private DcMotor intake_motor;//intake
    private Servo pusher_servo;//intake
    private Servo gate_servo;//intake
    private Servo scooper_servo;//intake
    private DcMotor hangOrientor_motor;//hang
    private DcMotor rightHang_motor;//hang
    private DcMotor leftHang_motor;//hang
    private Servo launcher_servo;//drone
    private DcMotor left_front_motor;//drive
    private DcMotor left_back_motor;//drive
    private DcMotor right_front_motor;//drive
    private DcMotor right_back_motor;//drive
    private final double MAX_POSITION = 220.0/270.0; // normalized for 270 degrees
    private final double MIN_POSITION = 0.0/270.0; // normalized for 0 degrees
    private boolean isAtMaxPosition = false;//checks if scooper has reached the intaking position (all the way down)
    private ElapsedTime runtime = new ElapsedTime(); //timer, used to remove sleeps
    private double checkEachInstance = 0;//keeps track of each iteration of outtake
    private double outtakeHold = 0;//finishes an outtake-run, instead of stopping mid-way when you release the button
    private double[] outtakeValues = {0.0, 0.995, 0.015, 0.565};//{gate, pusher, pusher, gate}
    //                              {gate, pusher, pusher, gate};
    private double WAIT_TIME = 85;// in milliseconds

    @Override
    public void init() {
        launcher_servo = hardwareMap.servo.get("launcher");
        launcher_servo.setDirection(Servo.Direction.FORWARD);

        hangOrientor_motor = hardwareMap.get(DcMotor.class, "Orient");
        rightHang_motor = hardwareMap.get(DcMotor.class, "RightHang");
        leftHang_motor = hardwareMap.get(DcMotor.class, "LeftHang");

        intake_motor = hardwareMap.get(DcMotor.class, "intake");
        pusher_servo = hardwareMap.servo.get("pusher");
        gate_servo = hardwareMap.servo.get("gate");
        scooper_servo= hardwareMap.servo.get("front");
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
        rightHang_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftHang_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        runtime.reset();
    }

    public void Intake() {
        if (gamepad2.a) {//forward intake
            intake_motor.setPower(-1);
            telemetry.addLine("Intake Is Spinning Forward.");
        } else if (gamepad2.b) {//backward intake
            intake_motor.setPower(0.4);
            telemetry.addLine("Intake Is Spinning Backwards.");
        } else {//stop intake
            intake_motor.setPower(0);
            telemetry.addData("scooper port: ", scooper_servo.getPortNumber());
        }

        if (gamepad2.y || outtakeHold == 1) { //outtake
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

        if (gamepad2.x) {//brings scooper down
            if (isAtMaxPosition == true) {
                isAtMaxPosition = false;
                scooper_servo.setPosition(MIN_POSITION);
            }
            else {
                isAtMaxPosition = true;
                scooper_servo.setPosition(MAX_POSITION);
            }
        }
    }





    public void DriveTrain() {//drive
        left_front_motor.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x);
        left_back_motor.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x);
        right_front_motor.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x);
        right_back_motor.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x);
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
        if (gamepad2.left_stick_y < -0.1 && gamepad2.right_stick_y < -0.1) {
            //extends hang when both joysticks are up
            rightHang_motor.setPower(1);
            if (gamepad2.dpad_down) {//brings hang down
                hangOrientor_motor.setPower(-1);
                sleep(525);
                hangOrientor_motor.setPower(0);
            }
            if (gamepad2.left_stick_y < -0.1 && gamepad2.right_stick_y < -0.1) {
                //extends hang when both joysticks are up
                rightHang_motor.setPower(1);
                leftHang_motor.setPower(0.775);
            } else if (gamepad2.left_stick_y > 0.1 && gamepad2.right_stick_y > 0.1) {
                //retract hang when both joysticks are up
                rightHang_motor.setPower(-1);
                leftHang_motor.setPower(-0.775);
            }
            leftHang_motor.setPower(0.775);
        } else if (gamepad2.left_stick_y > 0.1 && gamepad2.right_stick_y > 0.1) {
            //retract hang when both joysticks are up
            rightHang_motor.setPower(-1);
            leftHang_motor.setPower(-0.775);
        }
        else {
            //if nothing hang-actuator-related is happening, stop all hang movement
            rightHang_motor.setPower(0);
            leftHang_motor.setPower(0);
        }
        if (gamepad2.left_trigger > 0.08 && gamepad2.right_trigger > 0.08) {
            launcher_servo.setPosition(0); //launch drone is both triggers are pressed.
        }
    }


    @Override
    public void loop() {
        //runs game-elements and drive while printing telemetry
        Intake();
        DriveTrain();
        HangPlusDrone();
        telemetry.update();

    }
}