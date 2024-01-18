package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "IntakeMotorTest")
public class IntakeMotorTest extends OpMode {
    private DcMotor intake_motor;
    private boolean prev_gamepad2rightbumper = false;
    private boolean cur_gamepad2rightbumper = false;
    private boolean prev_gamepad2b= false;
    private boolean cur_gamepad2b= false;

    @Override
    public void init() {
        intake_motor = hardwareMap.get(DcMotor.class, "intake_motor");
        intake_motor.setDirection(DcMotor.Direction.REVERSE);
        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        prev_gamepad2rightbumper = cur_gamepad2rightbumper;
        cur_gamepad2rightbumper = gamepad2.right_bumper;
        prev_gamepad2b = cur_gamepad2b;
        cur_gamepad2b = gamepad2.b;

        if (cur_gamepad2rightbumper && prev_gamepad2rightbumper) {//forward intake
            intake_motor.setPower(0.995);
        } else if (cur_gamepad2b && prev_gamepad2b) {//reverse intake
            intake_motor.setPower(-0.65);
        }else {//stop intake
            intake_motor.setPower(0);
        }
    }
}
