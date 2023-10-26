package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "DrivetrainFinal", group = "iterativeOpMode")
public class DriverEnhancedPeriod extends OpMode {
    private DcMotor intake;//
    private Servo pusher;//
    private Servo gate;//
    private Servo Front;//
    private DcMotor hangOrientor;//
    private DcMotor rightHang;//
    private DcMotor leftHang;//
    private DcMotor left_front;//
    private DcMotor left_back;//
    private DcMotor right_front;//
    private DcMotor right_back;//
    private Servo launcher;//
    double OneValue = 1;
    private final double MIN_POSITION = 0.0; // assuming 0.0 is 0 degrees
    private final double MAX_POSITION = 240.0/270.0; // normalized for 270 degrees
    private final double LOWER_LIMIT = 220.0/270.0; // normalized for 250 degrees
    private boolean isAtMaxPosition = false;
    private int noLAG = 0;
    @Override
    public void init() {
        launcher = hardwareMap.servo.get("launcher");
        launcher.setDirection(Servo.Direction.REVERSE);

        hangOrientor = hardwareMap.get(DcMotor.class, "Orient");
        rightHang = hardwareMap.get(DcMotor.class, "RightHang");
        leftHang = hardwareMap.get(DcMotor.class, "LeftHang");

        hangOrientor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        pusher = hardwareMap.servo.get("pusher");
        gate = hardwareMap.servo.get("gate");
        Front = hardwareMap.servo.get("front");
        intake.setDirection(DcMotor.Direction.FORWARD);
        pusher.setDirection(Servo.Direction.FORWARD);

        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        right_back = hardwareMap.get(DcMotor.class, "right_back");
        right_front.setDirection(DcMotor.Direction.REVERSE);
        right_back.setDirection(DcMotor.Direction.REVERSE);

        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void Intake() {
        if (gamepad2.a) {
            intake.setPower(-1);
            telemetry.addLine("Intake Is Spinning Forward.");
        } else if (gamepad2.b) {
            intake.setPower(0.4);
            telemetry.addLine("Intake Is Spinning Backwards.");
        } else {
            intake.setPower(0);
            telemetry.addLine("Intake Stopped And Is Waiting...");
        }

        if (gamepad2.y) { //outtake
            gate.setPosition(-0.5);
            sleep(200);
            pusher.setPosition(1);
            sleep(200);
            pusher.setPosition(0.0);
            sleep(200);
            gate.setPosition(0.5);
            telemetry.addLine("Pixel Has Been Dropped.");
        }
        //we removed buffer because there were unnecessary lines of code(the if else statements)


        // When gamepad button 'a' is pressed, move to 270 degrees
        if (gamepad2.x) {
            Front.setPosition(MAX_POSITION);
            isAtMaxPosition = true; // Update the toggle state
        }

        // When gamepad button 'right_bumper' is pressed, alternate between 250 and 270 degrees
        if (gamepad2.right_bumper) {
            if (!isAtMaxPosition) {
                Front.setPosition(1);
                isAtMaxPosition = true;
            } else {
                Front.setPosition(0.65);
                isAtMaxPosition = false;
            }
        }
    }





    public void DriveTrain() {
        left_front.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x);
        left_back.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x);
        right_front.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x);
        right_back.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x);
    }




    public void HangPlusDrone() {
        if (gamepad2.dpad_up) {
            hangOrientor.setPower(0.5);
            sleep(900);
            hangOrientor.setPower(0);
        }
        if (gamepad2.dpad_down) {
            hangOrientor.setPower(-0.5);
            sleep(1050);
            hangOrientor.setPower(0);
        }
        if (gamepad2.left_stick_y < -0.1 && gamepad2.right_stick_y < -0.1) {
            rightHang.setPower(1);
            leftHang.setPower(0.775);
        } else if (gamepad2.left_stick_y > 0.1 && gamepad2.right_stick_y > 0.1) {
            rightHang.setPower(-1);
            leftHang.setPower(-0.775);
        }
        else {
            rightHang.setPower(0);
            leftHang.setPower(0);
        }
        if (gamepad2.dpad_left) {
            launcher.setPosition(-0.3);
        }
    }


    @Override
    public void loop() {
        Intake();
        DriveTrain();
        HangPlusDrone();
        telemetry.update();

    }
}
