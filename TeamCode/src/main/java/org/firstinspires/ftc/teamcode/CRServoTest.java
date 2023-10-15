package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous(name = "CRServoTest", group = "iterativeOpMode")
public class CRServoTest extends OpMode {

    public CRServo test;

    public CRServo test2;

    private void resetServos() {
        telemetry.addData("resetting servos", "");
        test.setPower(0);
        test2.setPower(0);
    }

    @Override
    public void init() {
        test = hardwareMap.get(CRServo.class, "test");
        test2 = hardwareMap.get(CRServo.class, "test2");
    }

    @Override
    public void loop() {




        if (gamepad1.b) {
            test.setPower(-1);
            test2.setPower(-1);
            telemetry.addData("outTakeServo1 current position", test.getPower());
            telemetry.update();
        }
        else if (gamepad1.a){
            test.setPower(1);
            test2.setPower(1);
            telemetry.addData("outTakeServo1 current position", test.getPower());
            telemetry.update();
        }
        else {
            resetServos();
        }
    }
}

