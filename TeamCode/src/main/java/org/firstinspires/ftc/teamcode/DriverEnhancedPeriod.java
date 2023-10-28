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
    private int customCaliberation = 1;
    private final double MAX_POSITION = 220.0/270.0; // normalized for 270 degrees
    private final double LOWER_LIMIT = 210.0/270.0; // normalized for 250 degrees
    private boolean isAtMaxPosition = false;

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

        if (gamepad2.x) {
            Front.setPosition(MAX_POSITION);
            isAtMaxPosition = true; // Update the toggle state
        }

        if (gamepad2.right_bumper) {
            customCaliberation = 1;
        }
        else {
            customCaliberation = 0;
        }

        if (gamepad2.right_bumper) {
            if (!isAtMaxPosition) {
                Front.setPosition(MAX_POSITION);
                isAtMaxPosition = true;
            } else {
                Front.setPosition(LOWER_LIMIT);
                isAtMaxPosition = false;
            }
        }
        else if (customCaliberation == 1) {
            if (isAtMaxPosition == true) {
                Front.setPosition(MAX_POSITION);
            } else {
                Front.setPosition(LOWER_LIMIT);
            }
        }
    }





    public void DriveTrain() {
        left_front.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x);
        left_back.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x);
        right_front.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x);
        right_back.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x);
    }




    public void HangPlusDrone() {
        if (gamepad2.dpad_up) {
            hangOrientor.setPower(1);
            sleep(325);
            hangOrientor.setPower(0);
        }
        if (gamepad2.dpad_down) {
            hangOrientor.setPower(-1);
            sleep(525);
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
        if (gamepad2.left_trigger > 0.08 && gamepad2.right_trigger > 0.08) {
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
