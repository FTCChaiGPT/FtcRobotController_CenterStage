package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="IntakeTest", group = "test")
public class IntakeTest extends OpMode {

    private DcMotor intake;


    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            intake.setPower(1);
        }
        else {
            intake.setPower(0);
        }
        if (gamepad1.b) {
            intake.setPower(-1);
        }
        else{
            intake.setPower(0);
        }
    }
}
