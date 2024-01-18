package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;


@TeleOp(name = "HelloWorld")
public class HelloWorld extends OpMode {

    Gamepad gamepad1;

    double speedForward = gamepad1.left_stick_y * 0.5;

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        telemetry.addData("Speed is ", speedForward);
        telemetry.update();
    }

    /*double speedForward = gamepad1.left_stick_y;
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        telemetry.addData("Speed is ", speedForward);
        telemetry.update();
    }*/

//    int age = 13;
//
//    int grade = 8;
//
//    String name = "Adheesh";
//    @Override
//    public void init() {
//
//    }
//
//    @Override
//    public void loop() {
//        telemetry.addData("My name is ", name);
//        telemetry.addData("My grade is ", grade);
//        telemetry.addData("My age is ", age);
//        telemetry.update();
//    }
}
