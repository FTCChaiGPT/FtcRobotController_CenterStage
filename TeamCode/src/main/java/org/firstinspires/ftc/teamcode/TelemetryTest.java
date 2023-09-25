package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="TelemetryTest", group = "test")
public class TelemetryTest extends OpMode {

    @Override
    public void init() {
        telemetry.addData("Test", "Works");
        telemetry.update();

    }

    @Override
    public void loop() {


    }
}