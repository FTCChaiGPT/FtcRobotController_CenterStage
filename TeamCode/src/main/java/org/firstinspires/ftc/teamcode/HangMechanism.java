package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Hang", group = "Driver")
public class HangMechanism extends OpMode {
    private DcMotor hangOrientor;//
    private DcMotor rightHang;//
    private DcMotor leftHang;//

    private Servo launcher;//


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

    }

    @Override
    public void loop() {
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

}

