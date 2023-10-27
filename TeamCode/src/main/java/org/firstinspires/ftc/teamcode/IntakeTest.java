package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="IntakeFinal" +
        "", group = "test")
public class IntakeTest extends LinearOpMode {
    private DcMotor intake;
    private Servo pusher;
    private Servo gate;
    private Servo Front;
    private final double MIN_POSITION = 0.0; // assuming 0.0 is 0 degrees
    private final double MAX_POSITION = 270.0/270.0; // normalized for 270 degrees
    private final double LOWER_LIMIT = 250.0/270.0; // normalized for 250 degrees

    // A flag to track the toggle state for 'b' button
    private boolean isAtMaxPosition = false;

    @Override
    public void runOpMode() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        pusher = hardwareMap.servo.get("pusher");
        gate = hardwareMap.servo.get("gate");
        Front = hardwareMap.servo.get("front");
        intake.setDirection(DcMotor.Direction.FORWARD);
        pusher.setDirection(Servo.Direction.FORWARD);
        waitForStart();

        // Set initial servo position
        Front.setPosition(MIN_POSITION);

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad2.a) {
                intake.setPower(-1);
                telemetry.addLine("Intake Is Spinning Forward.");
            } else if (gamepad2.b) {
                intake.setPower(0.4);
                telemetry.addLine("Intake Is Spinning Backwards.");
            } else {
                intake.setPower(0);
                telemetry.addLine("Intake Stopped And Is Waiting...");
            }

            if (gamepad2.y) { //outtake
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
            if (gamepad2.x) {
                Front.setPosition(MAX_POSITION);
                isAtMaxPosition = true; // Update the toggle state
            }

            // When gamepad button 'right_bumper' is pressed, alternate between 250 and 270 degrees
            if (gamepad2.right_bumper) {
                if (!isAtMaxPosition) {
                    Front.setPosition(1);
                    isAtMaxPosition = true;
                } else {
                    Front.setPosition(0.65);
                    isAtMaxPosition = false;
                }
            }
        }
    }
}
