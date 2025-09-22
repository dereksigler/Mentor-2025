package org.firstinspires.ftc.teamcode.SystemTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.SimpleColorLocator;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

@TeleOp(name = "Minimal Vision Demo (SDK 11)", group = "Vision")
public class MinimalVisionOpMode extends LinearOpMode {

    private VisionPortal visionPortal;
    private SimpleColorLocator locator;

    @Override
    public void runOpMode() {
        locator = new SimpleColorLocator();

        // Example: set bright yellow-ish range (tune as needed)
        locator.setHsvRange(new Scalar(20, 120, 120), new Scalar(35, 255, 255));

        // Webcam must match your Robot Config name
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        // VisionPortal (SDK 11). You can also use easyCreateWithDefaults(...)
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(locator)
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        telemetry.addLine("Vision initializing...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean seen = locator.hasTarget();
            Point c = locator.getCentroid();

            telemetry.addData("Has Target", seen);
            telemetry.addData("Centroid", "(%.1f, %.1f)", c.x, c.y);
            telemetry.addData("Area", "%.0f", locator.getArea());
            telemetry.addData("Frame", "%dx%d", locator.getWidth(), locator.getHeight());
            telemetry.update();

            // Quick on-field HSV presets
            if (gamepad1.a) {
                locator.setHsvRange(new Scalar(0, 120, 120), new Scalar(10, 255, 255));     // red-ish
            } else if (gamepad1.b) {
                locator.setHsvRange(new Scalar(100, 120, 120), new Scalar(130, 255, 255));  // blue-ish
            } else if (gamepad1.y) {
                locator.setHsvRange(new Scalar(20, 120, 120), new Scalar(35, 255, 255));    // yellow-ish
            }

            sleep(20);
        }

        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
