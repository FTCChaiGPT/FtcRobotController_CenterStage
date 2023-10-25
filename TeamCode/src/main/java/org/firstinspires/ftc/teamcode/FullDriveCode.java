package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="FullDriveCode", group = "test")
public class FullDriveCode extends LinearOpMode {
    private DcMotor intake;
    private Servo pusher;
    private Servo gate;
    private Servo frontServo;
    private DcMotor left_front;
    private DcMotor right_front;
    private DcMotor left_back;
    private DcMotor right_back;
    private DcMotor hangOrientor;//
    private DcMotor rightHang;//
    private DcMotor leftHang;//
    private Servo launcher;//

    double OneValue = 1;
    private final double MIN_POSITION = 0.0; // assuming 0.0 is 0 degrees
    private final double MAX_POSITION = 270.0/270.0; // normalized for 270 degrees
    private final double LOWER_LIMIT = 250.0/270.0; // normalized for 250 degrees

    // A flag to track the toggle state for 'b' button
    private boolean isAtMaxPosition = false;

    @Override
    public void runOpMode() {
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        right_back = hardwareMap.get(DcMotor.class, "right_back");
        intake = hardwareMap.get(DcMotor.class, "intake");
        pusher = hardwareMap.get(Servo.class, "pusher");
        gate = hardwareMap.get(Servo.class, "gate");
        frontServo = hardwareMap.get(Servo.class, "front");


        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pusher.setDirection(Servo.Direction.FORWARD);
        right_front.setDirection(DcMotor.Direction.REVERSE);
        right_back.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        waitForStart();

        // Set initial servo position
        frontServo.setPosition(MIN_POSITION);

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.a) {
                intake.setPower(-1);
                telemetry.addLine("Intake Is Spinning Forward.");
            } else if (gamepad1.b) {
                intake.setPower(0.4);
                telemetry.addLine("Intake Is Spinning Backwards.");
            } else {
                intake.setPower(0);
                telemetry.addLine("Intake Stopped And Is Waiting...");
            }

            if (gamepad1.y) { //outtake
                gate.setPosition(-0.5);
                sleep(200);
                pusher.setPosition(1);
                sleep(200);
                pusher.setPosition(0.0);
                sleep(200);
                gate.setPosition(0.5);
                telemetry.addLine("Pixel Has Been Dropped.");
            }
            //we removed buffer because there were unnecessary lines of code(the if else statements)


            // When gamepad button 'a' is pressed, move to 270 degrees
            if (gamepad1.x) {
                frontServo.setPosition(MAX_POSITION);
                isAtMaxPosition = true; // Update the toggle state
            }

            // When gamepad button 'right_bumper' is pressed, alternate between 250 and 270 degrees
            if (gamepad1.right_bumper) {
                if (!isAtMaxPosition) {
                    frontServo.setPosition(1);
                    isAtMaxPosition = true;
                } else {
                    frontServo.setPosition(0.65);
                    isAtMaxPosition = false;
                }

                // Pause briefly to prevent jittering due to rapid position changes
                sleep(500);
            }

            // Optional: give telemetry feedback
            telemetry.addData("Servo Position", frontServo.getPosition());
            telemetry.update();
        }

        DriveTrain();
        Hang();
    }
    public void DriveTrain() {
        left_front.setPower(gamepad2.left_stick_y + gamepad2.right_stick_x + gamepad2.left_stick_x);
        left_back.setPower(gamepad2.left_stick_y + gamepad2.right_stick_x - gamepad2.left_stick_x);
        right_front.setPower(gamepad2.left_stick_y - gamepad2.right_stick_x - gamepad2.left_stick_x);
        right_back.setPower(gamepad2.left_stick_y - gamepad2.right_stick_x + gamepad2.left_stick_x);
    }

    public void Hang() {
        boolean leftTrigger = gamepad1.left_trigger == 1;
        boolean rightTrigger = gamepad1.right_trigger == 1;
        if (leftTrigger && rightTrigger) {

            if (OneValue == -1) {
                rightHang.setPower(-0.775);
                leftHang.setPower(-1);
                sleep(5000);
                rightHang.setPower(0);
                leftHang.setPower(0);
                OneValue = 0;
            }
            if (OneValue == 1) {
                hangOrientor.setPower(0.5);
                sleep(1050);
                hangOrientor.setPower(0);
                sleep(1000);
                rightHang.setPower(0.775);
                leftHang.setPower(1);
                sleep(5000);
                rightHang.setPower(0);
                leftHang.setPower(0);
                hangOrientor.setPower(0.05);
                launcher.setPosition(-0.3);
                telemetry.addLine("Launched!");
                hangOrientor.setPower(0);
                sleep(1000);
                hangOrientor.setPower(0.5);
                sleep(1650);
                hangOrientor.setPower(0);
                sleep(1000);
                OneValue = -1;
            }

        }
        leftTrigger = gamepad1.left_trigger == 1;
        rightTrigger = gamepad1.right_trigger == 1;
    }

}
