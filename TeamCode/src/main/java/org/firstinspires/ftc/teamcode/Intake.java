package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="testintake", group = "test")
public class Intake extends LinearOpMode {

    private DcMotor intake;
    private Servo pusher;




    @Override
    public void runOpMode() throws InterruptedException {
        intake = hardwareMap.get(DcMotor.class, "intake");
        pusher = hardwareMap.servo.get("pusher");
        intake.setDirection(DcMotor.Direction.FORWARD);
        pusher.setDirection(Servo.Direction.FORWARD);
        waitForStart();
        while(opModeIsActive()) {

            if (gamepad1.a) {
                intake.setPower(-1);
                telemetry.addLine("Intake Is Spinning Forward.");
            } else if (gamepad1.b) {
                intake.setPower(0.4);
                telemetry.addLine("Intake Is Spinning Backwards.");
            } else {
                intake.setPower(0);
                telemetry.addLine("Intake Stopped And Is Waiting...");
            }

            if (gamepad1.x) {
                pusher.setPosition(1.25);
                sleep(500);
                pusher.setPosition(0.0);
                telemetry.addLine("Pixel Has Been Dropped.");
            }
        }
    }
}

