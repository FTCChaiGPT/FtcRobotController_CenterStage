package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class MyProgram extends LinearOpMode {
    private Servo myServo; // Declare a servo object

    @Override
    public void runOpMode(){
        // Initialize the servo object
        myServo = hardwareMap.get(Servo.class, "my_servo");

        myServo.setPosition(0.5);
    }
}

