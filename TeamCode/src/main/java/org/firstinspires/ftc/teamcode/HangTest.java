package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "HangOp", group = "Test")
public class HangTest extends OpMode {

    private Gamepad driverGamepad;

    DcMotor rightActuator;
    DcMotor leftActuator;
    DcMotor hangArm;

    @Override
    public void init() {
        rightActuator = hardwareMap.get(DcMotor.class, "rightActuator");
        leftActuator = hardwareMap.get(DcMotor.class, "leftAcutator");
        hangArm = hardwareMap.get(DcMotor.class, "hangArm");
        driverGamepad = gamepad1;

    }

    @Override
    public void loop() {
        if (driverGamepad.y){
            rightActuator.setPower(1);
            leftActuator.setPower(1);
        }
        else{
            rightActuator.setPower(0);
            leftActuator.setPower(0);
        }
        if (driverGamepad.a){
            rightActuator.setPower(-1);
            leftActuator.setPower(-1);
        }
        else{
            rightActuator.setPower(0);
            leftActuator.setPower(0);
        }
        if(driverGamepad.b){
            hangArm.setPower(1);
        }
        else{
            hangArm.setPower(0);
        }
        if(driverGamepad.x){
            hangArm.setPower(-1);
        }
        else{
            hangArm.setPower(0);
        }
    }
}