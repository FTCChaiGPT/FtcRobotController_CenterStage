package org.firstinspires.ftc.teamcode;

import android.os.Bundle;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class DriveOp extends OpMode {
    RobotLocation robotLocation;
    Gamepad gamepad1;


    @Override
    public void init() {
        robotLocation = new RobotLocation(0, 0);

    }

    @Override
    public void loop() {
        double change = robotLocation.changeX(gamepad1.left_stick_x);
        telemetry.addData("X is", change);
        telemetry.update();
        double change1 = robotLocation.changeY(gamepad1.left_stick_y);
        telemetry.addData("Y is", change1);
        telemetry.update();
    }
}
