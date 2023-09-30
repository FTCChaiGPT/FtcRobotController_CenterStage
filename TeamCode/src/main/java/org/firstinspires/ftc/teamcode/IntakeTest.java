package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="IntakeTest", group = "test")
public class IntakeTest extends LinearOpMode {

    private DcMotor intake;
    private Servo intakeServo;

    Gamepad gamepad1 = new Gamepad();


    @Override
    public void runOpMode() throws InterruptedException {
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        intake = hardwareMap.get(DcMotor.class, "intake");

        if (gamepad1.a) {
            intake.setPower(1);
            intakeServo.setPosition(1000);
        }
        else {
            intake.setPower(0);
        }
        if (gamepad1.b) {
            intake.setPower(-1);
            intakeServo.setPosition(0);
        }
        else{
            intake.setPower(0);
        }

    }


}
