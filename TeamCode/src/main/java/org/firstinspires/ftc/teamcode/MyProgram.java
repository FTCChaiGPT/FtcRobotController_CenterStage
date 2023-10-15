package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name="ServoTest", group = "test")
public class MyProgram extends OpMode {

    private long startTime;
    private Servo otServo2; // Outtake servo 2
    private Servo otServo3;

    @Override
    public void init() {
        // Initialize the servo object
        otServo2 = hardwareMap.get(Servo.class, "test3");
        otServo3 = hardwareMap.get(Servo.class, "test2");
        //telemetry.addData("Initializing hardware", otServo1.getDeviceName() + otServo2.getDeviceName());
        telemetry.update();
        startTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        // spin servos to target position when b is pressed
        if (gamepad1.b) {
            long currentTime = System.currentTimeMillis();
            otServo3.setPosition(0.5);
            otServo2.setPosition(0.5);
            telemetry.addData("outTakeServo1 current position", otServo3.getPosition());
            //telemetry.addData("outTakeServo2 current position", otServo2.getPosition());
            telemetry.update();
        }
        else if (gamepad1.a){
            otServo3.setPosition(-0.5);
            otServo2.setPosition(-0.5);
            telemetry.addData("outTakeServo1 current position", otServo3.getPosition());
            //telemetry.addData("outTakeServo2 current position", otServo2.getPosition());
            telemetry.update();
        }
        else {
            resetServos();
        }

    }

    private void resetServos() {
        telemetry.addData("resetting servos", "");
        otServo3.setPosition(0);
        otServo2.setPosition(0);
    }
}