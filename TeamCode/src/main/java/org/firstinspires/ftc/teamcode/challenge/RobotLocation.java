package org.firstinspires.ftc.teamcode.challenge;

public class RobotLocation {
    private double x;
    private double y;
    public RobotLocation(double x, double y){
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }



    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public String toString(){

        return "x::"+x+"y::"+ y;
    }




}
