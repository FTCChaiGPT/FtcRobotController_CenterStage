package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "DroneTest", group = "test")
public class Drone extends LinearOpMode {
    private Servo launcher;




    @Override
    public void runOpMode() throws InterruptedException {
        launcher = hardwareMap.servo.get("launcher");
        launcher.setDirection(Servo.Direction.FORWARD);

        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.a) {
                launcher.setPosition(0.25);
                telemetry.addLine("Launched!");

            }
        }

    }
}
