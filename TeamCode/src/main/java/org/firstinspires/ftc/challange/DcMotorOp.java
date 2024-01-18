package org.firstinspires.ftc.challange;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DcMotorOp extends OpMode {
    DcMotor dcmotor;

    private ElapsedTime timer = new ElapsedTime();



    @Override
    public void init() {
        dcmotor = hardwareMap.get(DcMotor.class, "dcMotor");
    }

    @Override
    public void loop() {
        turn(0.5, 3.0);
    }
    void turn(double power, double time){
        dcmotor.setPower(power);
        if (timer.seconds() > time){
            dcmotor.setPower(0);
            timer.reset();
        }

    }
}
