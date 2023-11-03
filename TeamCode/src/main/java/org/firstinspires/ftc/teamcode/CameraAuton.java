package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;



public class CameraAuton {

    public CameraAuton(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }
    boolean pixel_found;
    double pixel_confidence;
    double pixel_x_coord;
    double pixel_y_coord;
    TfodProcessor myTfodProcessor;
    VisionPortal myVisionPortal;

    private DcMotor intake;

    Telemetry telemetry;

    HardwareMap hardwareMap;


    public void runOpMode() {

        Init();


    }

    public void Init(){
        // Initialize TfodProcessor and VisionPortal

        TfodProcessor.Builder myTfodProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;

        // First, create a TfodProcessor.Builder.
        myTfodProcessorBuilder = new TfodProcessor.Builder();
        // Create a TfodProcessor by calling build.
        myTfodProcessor = myTfodProcessorBuilder.build();
        // Next, create a VisionPortal.Builder and set attributes related to the camera.
        myVisionPortalBuilder = new VisionPortal.Builder();

        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Add myTfodProcessor to the VisionPortal.Builder.
        myVisionPortalBuilder.addProcessor(myTfodProcessor);
        // Create a VisionPortal by calling build.
        myVisionPortal = myVisionPortalBuilder.build();

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        pixel_found = false;
    }

    public boolean detect() {
        // Put run blocks here.
        pixel_found = false;
        // Put loop blocks here.
        PixelInfo pixelInfo = whitepixel();
        pixel_found = pixelInfo.pixel_found;
        pixel_confidence = pixelInfo.pixel_confidence;
        pixel_x_coord = pixelInfo.pixel_x;
        pixel_y_coord = pixelInfo.pixel_y;
        //telemetryTfod();
        // Push telemetry to the Driver Station.
        telemetry.addLine("");
        telemetry.addData("pixel_found", pixel_found);
        telemetry.addData("pixel_confidence", pixel_confidence);
        telemetry.addData("pixel_x", pixel_confidence);
        telemetry.addData("pixel_y", pixel_confidence);
        telemetry.update();
        // Share the CPU.


        if (pixel_confidence >= 0.85) {
            pixel_found = true;
        }
        return pixel_found;
    }

    class PixelInfo {
        boolean pixel_found;
        double pixel_confidence;
        double pixel_x;
        double pixel_y;

        public PixelInfo(boolean found, double confidence, double x, double y) {
            this.pixel_found = found;
            this.pixel_confidence = confidence;
            this.pixel_x = x;
            this.pixel_y = y;
        }
    }


    private PixelInfo whitepixel() {
        List<Recognition> myTfodRecognitions;
        Recognition myTfodRecognition;
        float x = 0;
        float y = 0;
        float pixel_conf;
        boolean pixelCameraFound = false;

        // Get a list of recognitions from TFOD.
        myTfodRecognitions = myTfodProcessor.getRecognitions();
        telemetry.addData("# Objects Detected", JavaUtil.listLength(myTfodRecognitions));
        // Iterate through list and call a function to display info for each recognized object.
        for (Recognition myTfodRecognition_item : myTfodRecognitions) {
            myTfodRecognition = myTfodRecognition_item;
            // Display info about the recognition.
            telemetry.addLine("");
            // Display label and confidence.
            // Display the label and confidence for the recognition.

            if (myTfodRecognition.getLabel().equals("Pixel") && myTfodRecognition.getConfidence() >= 0.85 && !pixelCameraFound) {
                // we are going to take only pixels with more than 80% confidence
                pixel_conf = myTfodRecognition.getConfidence();
                telemetry.addData("Image", myTfodRecognition.getLabel() + " (" + JavaUtil.formatNumber(pixel_conf * 100, 0) + " % Conf.)");
                // Display position.
                x = (myTfodRecognition.getLeft() + myTfodRecognition.getRight()) / 2;
                y = (myTfodRecognition.getTop() + myTfodRecognition.getBottom()) / 2;
                telemetry.addData("- Position", JavaUtil.formatNumber(x, 0) + ", " + JavaUtil.formatNumber(y, 0));

                pixelCameraFound = true;
                return new PixelInfo(true, pixel_conf, x, y);
            }
        }
        return new PixelInfo(false, 0.0, x, y); // Return a default value if no pixel is found
    }


    /**
     * Display info (using telemetry) for a detected object
     */
    private void telemetryTfod() {
        List<Recognition> myTfodRecognitions;
        Recognition myTfodRecognition;
        float x;
        float y;

        // Get a list of recognitions from TFOD.
        myTfodRecognitions = myTfodProcessor.getRecognitions();
        telemetry.addData("# Objects Detected", JavaUtil.listLength(myTfodRecognitions));
        // Iterate through list and call a function to display info for each recognized object.
        for (Recognition myTfodRecognition_item : myTfodRecognitions) {
            myTfodRecognition = myTfodRecognition_item;
            // Display info about the recognition.
            telemetry.addLine("");
            // Display label and confidence.
            // Display the label and confidence for the recognition.
            telemetry.addData("Image", myTfodRecognition.getLabel() + " (" + JavaUtil.formatNumber(myTfodRecognition.getConfidence() * 100, 0) + " % Conf.)");
            // Display position.
            x = (myTfodRecognition.getLeft() + myTfodRecognition.getRight()) / 2;
            y = (myTfodRecognition.getTop() + myTfodRecognition.getBottom()) / 2;
            // Display the position of the center of the detection boundary for the recognition
            telemetry.addData("- Position", JavaUtil.formatNumber(x, 0) + ", " + JavaUtil.formatNumber(y, 0));
            // Display size
            // Display the size of detection boundary for the recognition
            telemetry.addData("- Size", JavaUtil.formatNumber(myTfodRecognition.getWidth(), 0) + " x " + JavaUtil.formatNumber(myTfodRecognition.getHeight(), 0));
        }
    }



}
