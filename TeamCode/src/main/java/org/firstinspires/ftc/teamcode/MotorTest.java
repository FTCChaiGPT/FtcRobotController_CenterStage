package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="MotorTest", group = "Test" )
public class MotorTest extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    DcMotor motor;

    @Override
    public void init(){
        motor = hardwareMap.get(DcMotor.class, "motor");

    }

    @Override
    public void loop(){

        motor.setPower(1);

        if (runtime.seconds()>5){
            motor.setPower(1);
            if (runtime.seconds()>5){
                motor.setPower(0);

            }
        }


    }

}
