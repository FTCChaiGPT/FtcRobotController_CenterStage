package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="MotorTest", group = "Test" )
public class MotorTest extends OpMode {

    DcMotor motor;

    @Override
    public void init(){
        motor = hardwareMap.get(DcMotor.class, "motor");

    }

    @Override
    public void loop(){

        motor.setPower(1);

    }

}
