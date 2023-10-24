package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "HangMechanism")
public class Hang extends LinearOpMode {
    private DcMotor HangOrientor;
    private DcMotor RightHang;
    private DcMotor LeftHang;

    public void initialize() {
        HangOrientor = hardwareMap.get(DcMotor.class, "Orient");
        RightHang = hardwareMap.get(DcMotor.class, "RightHang");
        LeftHang = hardwareMap.get(DcMotor.class, "LeftHang");

        HangOrientor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void HangExtention(int Lift, double DirectionOneValue) {
        if (Lift == 1) {
            HangOrientor.setPower(1);
            sleep(2000);
            HangOrientor.setPower(0);
        }
        double RightValue = 0.85 * DirectionOneValue;
        double LeftValue = 0.95 * DirectionOneValue;
        RightHang.setPower(RightValue);
        LeftHang.setPower(LeftValue);
        sleep(4850);
        RightHang.setPower(0);
        LeftHang.setPower(0);

    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();


        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.a) {
                HangExtention(1, 1);
            }
            if(gamepad1.b) {
                HangExtention(0, -1);
            }
        }



    }
}
