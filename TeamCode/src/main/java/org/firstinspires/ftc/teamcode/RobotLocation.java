package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class RobotLocation {
    private double x;
    private double y;
    public RobotLocation(double x, double y){
        this.x = x;
        this.y = y;
    }

    public double changeX(double a) {
        this.x = x;
        x = a;
        return x;
    }
    public double changeY(double y) {
        this.y = y;
        y = y;
        return y;
    }
}
