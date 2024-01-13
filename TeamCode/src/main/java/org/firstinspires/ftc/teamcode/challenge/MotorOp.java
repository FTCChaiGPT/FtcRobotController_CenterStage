package org.firstinspires.ftc.teamcode.challenge;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorOp extends OpMode {
    DcMotor dcMotor;

    @Override
    public void init() {
        dcMotor = hardwareMap.get(DcMotor.class, "motor");
    }

    @Override
    public void loop() {
        // Run the motor at power 0.5 for 3 seconds
        // After 3 seconds stop the motor

    }

    private void runMotor(double power, long time) throws InterruptedException {
        dcMotor.setPower(power);
        sleep(time);
        dcMotor.setPower(0); //stop
    }
}
