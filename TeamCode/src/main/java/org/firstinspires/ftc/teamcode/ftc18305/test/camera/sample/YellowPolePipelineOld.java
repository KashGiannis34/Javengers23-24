package org.firstinspires.ftc.teamcode.ftc18305.test.camera.sample;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class YellowPolePipelineOld extends OpenCvPipeline {
    Telemetry telemetry;

    List<MatOfPoint> yellowContours = new ArrayList<>();
    double k = 1;
    List <MatOfPoint> yellowRects = new ArrayList<>();
    List <Double> distance = new ArrayList<>();
    private final Mat ycrcbMat       = new Mat();
    private final Mat binaryMat      = new Mat();
    private final Mat maskedInputMat = new Mat();
    Mat morphedThreshold = new Mat();
    Scalar RED = new Scalar(255, 0, 0);
    Scalar GREEN = new Scalar(0, 255, 0);
    private Rect yellowRect;
    private MatOfPoint biggestYellowContour;

//    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(15, 15));
//    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(15, 15));

    public YellowPolePipelineOld() {
        biggestYellowContour = new MatOfPoint();
        yellowRect = new Rect();
    }

//    public YellowPolePipeline(Telemetry telemetry) {
//        this.telemetry = telemetry;
//        biggestYellowContour = new MatOfPoint();
//        yellowRect = new Rect();
//    }

//    void morphMask(Mat input, Mat output)
//    {
//        /*
//         * Apply some erosion and dilation for noise reduction
//         */
//
//        Imgproc.erode(input, output, erodeElement);
//        Imgproc.erode(output, output, erodeElement);
//
//        Imgproc.dilate(output, output, dilateElement);
//        Imgproc.dilate(output, output, dilateElement);
//    }

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(ycrcbMat, ycrcbMat, 2);
        Imgproc.threshold(ycrcbMat, binaryMat, 100, 110, Imgproc.THRESH_BINARY_INV);
        //morphMask(binaryMat, morphedThreshold);

        yellowContours.clear();
        yellowRects.clear();

        Imgproc.findContours(morphedThreshold, yellowContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(input, yellowContours, -1, RED);

//        for (int i = 0; i < yellowContours.size(); i++){
//            for (int j = 0; j < 3; j++) {
//                distance.add((double)yellowContours.get(i).height());
//                telemetry.addData("yellow contour width: ", yellowContours.get(i).height());
//                telemetry.update();
//            }
//        }

        biggestYellowContour = Collections.max(yellowContours, (t0, t1) -> {
            return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
        });
        yellowRect = Imgproc.boundingRect(biggestYellowContour);
        Imgproc.rectangle(input, yellowRect, new Scalar(0, 0, 255), 3);
        Imgproc.putText(input, "Rect X: " + yellowRect.x + " Y: " + yellowRect.y, new Point(5, 715), 0, 1.5, new Scalar(255, 255, 255), 2);

        return input;
    }
}
