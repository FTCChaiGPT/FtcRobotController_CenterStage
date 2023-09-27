package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "HangOp", group = "Test")
public class HangTest extends OpMode {

    private Gamepad driverGamepad;

    DcMotor rightActuator;
    DcMotor leftAcutator;
    DcMotor hangArm;

    @Override
    public void init() {
        rightActuator = hardwareMap.get(DcMotor.class, "rightActuator");
        leftAcutator = hardwareMap.get(DcMotor.class, "leftAcutator");
        hangArm = hardwareMap.get(DcMotor.class, "hangArm");
        driverGamepad = gamepad1;

    }

    @Override
    public void loop() {
        if (driverGamepad.a){
            rightActuator.setPower(0.5);
            leftAcutator.setPower(0.5);
        }
        else{
            rightActuator.setPower(0);
            leftAcutator.setPower(0);
        }
        if(driverGamepad.b){
            hangArm.setPower(0.5);
        }
        else{
            hangArm.setPower(0);
        }
        if(driverGamepad.x){
            hangArm.setPower(-0.5);
        }
        else{
            hangArm.setPower(0);
        }
    }
}