package org.firstinspires.ftc.teamcode.challenge;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


public class TestDriveOp extends OpMode {
    RobotLocation robotLocation;

    @Override
    public void init() {
        robotLocation = new RobotLocation(0,0);
    }

    @Override
    public void loop() {
        // based on gamepad x value , set x for robotlocation
        //based on gamepad y value, set y for robotlocation
        // send robotlocation x and y to telemetry
        robotLocation.setX(gamepad1.left_stick_x);
        telemetry.addData("RobotLocation", robotLocation);
    }
}
