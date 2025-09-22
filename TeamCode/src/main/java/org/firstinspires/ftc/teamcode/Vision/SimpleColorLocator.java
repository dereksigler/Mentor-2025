package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.RectF;

import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/** Finds the largest blob in an HSV range and overlays bbox/centroid via Canvas. */
public class SimpleColorLocator implements VisionProcessor {

    // Tunable HSV bounds (OpenCV H:0–179, S/V:0–255)
    private volatile Scalar lower = new Scalar(20, 80, 80);   // yellow-ish default
    private volatile Scalar upper = new Scalar(35, 255, 255);

    // Reused Mats
    private final Mat hsv = new Mat();
    private final Mat mask = new Mat();
    private final Mat hierarchy = new Mat();

    // Results (accessed by OpMode)
    private volatile boolean hasTarget = false;
    private volatile Point centroid = new Point(-1, -1);
    private volatile double area = 0.0;
    private volatile Rect bbox = new Rect();

    // Frame size
    private volatile int width = 0, height = 0;

    // Paint for overlays
    private final Paint boxPaint = new Paint();
    private final Paint dotPaint = new Paint();
    private final Paint ringPaint = new Paint();

    public SimpleColorLocator() {
        boxPaint.setStyle(Paint.Style.STROKE);
        boxPaint.setStrokeWidth(4f);

        dotPaint.setStyle(Paint.Style.FILL);

        ringPaint.setStyle(Paint.Style.STROKE);
        ringPaint.setStrokeWidth(2f);
    }

    /**
     * Change HSV detection window at runtime.
     */
    public void setHsvRange(Scalar lower, Scalar upper) {
        this.lower = lower;
        this.upper = upper;
    }

    public boolean hasTarget() {
        return hasTarget;
    }

    public Point getCentroid() {
        return centroid;
    }

    public double getArea() {
        return area;
    }

    public Rect getBoundingBox() {
        return bbox;
    }

    public int getWidth() {
        return width;
    }

    public int getHeight() {
        return height;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        this.width = width;
        this.height = height;
    }


    public void deinit() {
        hsv.release();
        mask.release();
        hierarchy.release();
    }

    @Override
    public @Nullable Object processFrame(Mat rgbInput, long captureTimeNanos) {
        // Convert to HSV
        Imgproc.cvtColor(rgbInput, hsv, Imgproc.COLOR_RGB2HSV);

        // Threshold
        Core.inRange(hsv, lower, upper, mask);

        // Clean up small specks
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new org.opencv.core.Size(5, 5));
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double bestArea = 0.0;
        Rect bestRect = new Rect();
        Point bestCentroid = new Point(-1, -1);

        for (MatOfPoint c : contours) {
            double a = Imgproc.contourArea(c);
            if (a > bestArea) {
                bestArea = a;
                Rect r = Imgproc.boundingRect(c);
                bestRect = r;
                bestCentroid = new Point(r.x + r.width / 2.0, r.y + r.height / 2.0);
            }
            c.release();
        }

        area = bestArea;
        bbox = bestRect;
        centroid = bestCentroid;
        hasTarget = bestArea > 200.0; // tweak for your scene

        // You can return a lightweight
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas,
                            int onscreenWidth,
                            int onscreenHeight,
                            float scaleBmpPxToCanvasPx,
                            float scaleCanvasDensity,
                            @Nullable Object userContext) {
        if (!hasTarget) return;

        // Choose colors (ARGB). You can theme this as you like.
        boxPaint.setARGB(255, 0, 255, 0);
        dotPaint.setARGB(255, 255, 255, 255);
        ringPaint.setARGB(255, 0, 0, 0);

        // Scale OpenCV pixel coords -> Canvas coords
        float sx = scaleBmpPxToCanvasPx;
        float cx = (float) (centroid.x * sx);
        float cy = (float) (centroid.y * sx);

        // Draw bbox
        Rect r = bbox;
        RectF rf = new RectF(r.x * sx, r.y * sx, (r.x + r.width) * sx, (r.y + r.height) * sx);
        canvas.drawRect(rf, boxPaint);

        // Draw centroid (filled + ring)
        canvas.drawCircle(cx, cy, 6 * scaleCanvasDensity, dotPaint);
        canvas.drawCircle(cx, cy, 10 * scaleCanvasDensity, ringPaint);
    }
}