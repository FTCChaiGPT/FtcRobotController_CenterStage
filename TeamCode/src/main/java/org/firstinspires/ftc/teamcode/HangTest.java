package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "HangOp", group = "Test")
public class HangTest extends OpMode {

    DcMotor RightHang;
    DcMotor LeftHang;
    DcMotor RightLift;
    DcMotor LeftLift;
    @Override
    public void init() {
        RightHang = hardwareMap.get(DcMotor.class, "RightHang");
        LeftHang = hardwareMap.get(DcMotor.class, "LeftHang");
        LeftHang.setDirection(DcMotor.Direction.REVERSE);
        RightLift = hardwareMap.get(DcMotor.class, "RightLift");
        RightLift.setDirection(DcMotor.Direction.REVERSE);
        LeftLift = hardwareMap.get(DcMotor.class, "LeftLift");
    }

    @Override
    public void loop() {
        RightLift.setTargetPosition(45);
        LeftLift.setTargetPosition(45);

        RightHang.setPower(1);
        LeftHang.setPower(1);
        sleep(4650);
        RightHang.setPower(0);
        LeftHang.setPower(0);

    }
}