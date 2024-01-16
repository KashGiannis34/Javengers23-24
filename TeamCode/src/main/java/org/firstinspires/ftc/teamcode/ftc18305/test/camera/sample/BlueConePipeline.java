package org.firstinspires.ftc.teamcode.ftc18305.test.camera.sample;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

public class BlueConePipeline extends OpenCvPipeline {

    protected double centerX;
    protected double centerY;

    public int minThreshold, maxThreshold;
    private Mat blueThreshold;

    private Mat matYCrCb;
    private Mat blueChannel;

    private List<MatOfPoint> blueContours;
    private MatOfPoint biggestBlueContour;
    private Rect blueRect;

    public BlueConePipeline() {
        matYCrCb = new Mat();
        blueChannel = new Mat();
        blueThreshold = new Mat();
        blueContours = new ArrayList<MatOfPoint>();
        biggestBlueContour = new MatOfPoint();
        blueRect = new Rect();

        minThreshold = 155;
        maxThreshold = 200;
    }

    @Override
    public void init(Mat mat) {
        super.init(mat);
        int imageWidth = mat.width();
        int imageHeight = mat.height();

        centerX = ((double) imageWidth / 2) - 0.5;
        centerY = ((double) imageHeight / 2) - 0.5;
    }

    public boolean filterContours(MatOfPoint contour) {
        return Imgproc.contourArea(contour) > 30;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, matYCrCb, Imgproc.COLOR_RGB2YCrCb);

        Core.extractChannel(matYCrCb, blueChannel, 2);

        // Blue threshold
        Imgproc.threshold(blueChannel, blueThreshold, minThreshold, maxThreshold, Imgproc.THRESH_BINARY);

        blueContours.clear();

        Imgproc.findContours(blueThreshold, blueContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        blueContours = blueContours.stream().filter(i -> {
            boolean appropriateAspect = ((double) Imgproc.boundingRect(i).height / Imgproc.boundingRect(i).width > 1)
                    && ((double) Imgproc.boundingRect(i).height / Imgproc.boundingRect(i).width < 5);
            return filterContours(i) && appropriateAspect;
        }).collect(Collectors.toList());

        Imgproc.drawContours(input, blueContours, -1, new Scalar(255, 255, 0));

        if (!blueContours.isEmpty()) {
            // Comparing width instead of area because wobble goals that are close to the camera tend to have a large area
            biggestBlueContour = Collections.max(blueContours, (t0, t1) -> {
                return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
            });
            blueRect = Imgproc.boundingRect(biggestBlueContour);
            Imgproc.rectangle(input, blueRect, new Scalar(0, 0, 255), 3);
            Imgproc.putText(input, "Rect X: " + getBlueRect().x + " Y: " + getBlueRect().y + " , Center-X:" + (getBlueRect().x + getBlueRect().width/2), new Point(5, 715), 0, 1.5, new Scalar(255, 255, 255), 2);
        } else {
            blueRect = null;
        }
        return input;
    }

    public Rect getBlueRect() {
        return blueRect;
    }

    public boolean isBlueVisible() {
        return (blueRect != null);
    }

    public Point getCenterofRect(Rect rect) {
        if (rect == null) {
            return new Point(centerX, centerY);
        }
        return new Point(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);
    }

    public Point getCenter() {
        return new Point(centerX, centerY);
    }

    public double getCenterX() {
        return centerX;
    }
}
