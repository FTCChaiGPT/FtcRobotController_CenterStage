package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="pranavdrive", group = "test")
public class PranavDrive extends LinearOpMode {

    private DcMotor intake;
    private Servo pusher;
    private Servo gate;
    private Servo Front;




    @Override
    public void runOpMode() throws InterruptedException {
        intake = hardwareMap.get(DcMotor.class, "intake");
        pusher = hardwareMap.servo.get("pusher");
        gate = hardwareMap.servo.get("gate");
        Front = hardwareMap.servo.get("front");
        intake.setDirection(DcMotor.Direction.FORWARD);
        pusher.setDirection(Servo.Direction.FORWARD);
        waitForStart();

        Front.setPosition(0.0);
        Front.setPosition(3.0);
        Front.setPosition(-0.0);

        int one = 2;
        int buffer = 1;
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

            if (gamepad1.y) {
                gate.setPosition(0.5);
                sleep(200);
                pusher.setPosition(1);
                sleep(200);
                pusher.setPosition(0.0);
                sleep(200);
                gate.setPosition(-0.5);
                telemetry.addLine("Pixel Has Been Dropped.");
            }

            if (buffer == 0) {
                buffer = 1;
            }
            else {
                if (gamepad1.right_bumper) {
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
    }
}