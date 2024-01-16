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

public class YellowPolePipeline extends OpenCvPipeline {

    protected double centerX;
    protected double centerY;

    public int minThreshold, maxThreshold;
    private Mat threshold;

    private Mat matYCrCb;
    private Mat channel;

    private List<MatOfPoint> contours;
    private MatOfPoint biggestContour;
    private Rect rect;

    public YellowPolePipeline() {
        matYCrCb = new Mat();
        channel = new Mat();
        threshold = new Mat();
        contours = new ArrayList<MatOfPoint>();
        biggestContour = new MatOfPoint();
        rect = new Rect();

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

        Core.extractChannel(matYCrCb, channel, 1);

        // threshold
        Imgproc.threshold(channel, threshold, minThreshold, maxThreshold, Imgproc.THRESH_BINARY);
        contours.clear();
        Imgproc.findContours(threshold, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        contours = contours.stream().filter(i -> {
            boolean appropriateAspect = ((double) Imgproc.boundingRect(i).height / Imgproc.boundingRect(i).width > 1)
                    && ((double) Imgproc.boundingRect(i).height / Imgproc.boundingRect(i).width < 5);
            return filterContours(i) && appropriateAspect;
        }).collect(Collectors.toList());

        Imgproc.drawContours(input, contours, -1, new Scalar(255, 255, 0));

        if (!contours.isEmpty()) {
            // Comparing width instead of area because wobble goals that are close to the camera tend to have a large area
            biggestContour = Collections.max(contours, (t0, t1) -> {
                return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
            });
            rect = Imgproc.boundingRect(biggestContour);
            Imgproc.rectangle(input, rect, new Scalar(0, 0, 255), 3);
            Imgproc.putText(input, "Rect X: " + getRect().x + " Y: " + getRect().y + " , Center-X:" + (getRect().x + getRect().width/2), new Point(5, 715), 0, 1.5, new Scalar(255, 255, 255), 2);
        } else {
            rect = null;
        }
        return input;
    }

    public Rect getRect() {
        return rect;
    }

    public boolean isYellowPoleVisible() {
        return (rect != null);
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
