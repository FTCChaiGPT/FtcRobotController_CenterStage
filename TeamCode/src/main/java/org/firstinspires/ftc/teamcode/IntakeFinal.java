package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;



@TeleOp(name="intake", group = "test")
public class IntakeFinal extends LinearOpMode {

    private DcMotor intake;
    private Servo pusher;
    private Servo gate;
    private Servo Front;


    private static final double COUNTS_PER_MOTOR_REV = 756; //Number of encoder counts per motor revolution (1440)
    private static final double WHEEL_DIAMETER_INCHES = 5.5;
    private  static final double GEAR_RATIO = 1.0;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * GEAR_RATIO)/(WHEEL_DIAMETER_INCHES * Math.PI);

    private static final double ROBOT_WIDTH_INCHES = 27.4; // The distance between the wheels on opposite sides




    @Override
    public void runOpMode() throws InterruptedException {
        intake = hardwareMap.get(DcMotor.class, "intake");
        Front = hardwareMap.get(Servo.class, "front");

        //pusher.setDirection(Servo.Direction.FORWARD);
        waitForStart();
        while(opModeIsActive()) {

            if (gamepad1.x){
              intake();
            }
            else{
              intake.setPower(0);
              Front.setPosition(0);
            }

            //intake.setPower(-0.5);
            /*telemetry.addData("Servo position: ",Front.getPosition());
            telemetry.update();
            intake();
            telemetry.addData("Servo position: ",Front.getPosition());
            telemetry.update();*/
            //intake(30);
            //sleep(500);
            //Front.setPosition(0);

            /*if (gamepad1.x) {
                for (int i = 0; i<= 10; i++) {
                    if (i < 10) {
                        intake.setPower(0.5);
                        Front.setPosition(30);
                        sleep(500);
                        Front.setPosition(0);
                    }
                    else{
                        intake.setPower(0);
                        break;
                    }
                }
            }*/
        }
    }
    public void intake() {
        intake.setPower(-0.5);
        Front.setPosition(0.0);
        Front.setPosition(-0.025);
        sleep(2000);
        Front.setPosition(0.025);
    }
    private void setMotorPower(double x) {
        intake.setPower(x);
    }
    public void stopMotor() {
        setMotorPower(0);
        resetEncoders();
    }
    private void resetEncoders() {
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
