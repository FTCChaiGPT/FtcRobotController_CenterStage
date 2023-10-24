package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "DriverEnhanced", group = "iterativeOpMode")
public class MecanumDrive extends OpMode {
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
        int one = 2;
        int buffer = 1;
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

        if (gamepad2.y) {
            gate.setPosition(-0.5);
            sleep(200);
            pusher.setPosition(1);
            sleep(200);
            pusher.setPosition(0.0);
            sleep(200);
            gate.setPosition(0.5);
            telemetry.addLine("Pixel Has Been Dropped.");
        }

        if (buffer == 0) {
            buffer = 1;
        }
        else {
            if (gamepad2.right_bumper) {
                if (buffer == 1) {
                    sleep(30);
                    Front.setPosition(-0.3);
                    sleep(345);
                    Front.setPosition(0.3 * one);
                    sleep(70);
                    telemetry.addLine("Pixel Has Been Dropped.");
                }
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
        boolean leftTrigger = gamepad1.left_trigger == 1;
        boolean rightTrigger = gamepad1.right_trigger == 1;
        if (leftTrigger && rightTrigger) {
            if (OneValue == -1) {
                rightHang.setPower(-0.775);
                leftHang.setPower(-1);
                sleep(5000);
                rightHang.setPower(0);
                leftHang.setPower(0);
                OneValue = 0;
            }
            if (OneValue == 1) {
                hangOrientor.setPower(0.5);
                sleep(1050);
                hangOrientor.setPower(0);
                sleep(1000);
                rightHang.setPower(0.775);
                leftHang.setPower(1);
                sleep(5000);
                rightHang.setPower(0);
                leftHang.setPower(0);
                hangOrientor.setPower(0.05);
                launcher.setPosition(-0.3);
                telemetry.addLine("Launched!");
                hangOrientor.setPower(0);
                sleep(1000);
                hangOrientor.setPower(0.5);
                sleep(1650);
                hangOrientor.setPower(0);
                sleep(1000);
                OneValue = -1;
            }

        }
        leftTrigger = gamepad1.left_trigger == 1;
        rightTrigger = gamepad1.right_trigger == 1;
    }


    @Override
    public void loop() {
        DriveTrain();
        Intake();
        HangPlusDrone();
        telemetry.update();


    }
}