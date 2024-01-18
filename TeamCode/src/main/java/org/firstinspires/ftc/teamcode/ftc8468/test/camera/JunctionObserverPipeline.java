package org.firstinspires.ftc.teamcode.ftc8468.test.camera;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class JunctionObserverPipeline extends OpenCvPipeline {


    public JunctionObserverPipeline(){

    }
    public Mat processFrame(Mat input) {

        Mat mat = new Mat();

        //mat turns into HSV Value
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if(mat.empty()){
            return input;
        }

        Scalar lowHSV = new Scalar(20,70,80); //lenient lower bound HSV for yellow
        Scalar highHSV = new Scalar(32,255,255); //lenient higher bound HSV for yellow

        Mat thresh = new Mat();

        //get a black and white image of all yellow objects
        Core.inRange(mat, lowHSV, highHSV, thresh);

        Mat masked = new Mat();
        //color the white portion of thresh in with HSV from Mat
        //output into masked
        Core.bitwise_and(mat,mat,masked,thresh);
        //calculate the average HSV values of the white thresh values
        Scalar average = Core.mean(masked, thresh);

        Mat scaledMask = new Mat();
        //scale the average saturation to 150
        masked.convertTo(scaledMask,-1,150/average.val[1],0);

        Scalar strictLowHSV = new Scalar(0,150,100); //strict lower bound HSV for yellow
        Scalar strictHighHSV = new Scalar(255,255,255); //strict higher bound HSV for yellow

        Mat scaledThresh = new Mat();

        //apply strict HSV Filter onto scaledMask to get rid of any yellow other than pole
        Core.inRange(scaledMask,strictLowHSV, strictHighHSV, scaledThresh);

        Mat finalMask = new Mat();
        //color in scaledThresh with HSV(for showing results)
        Core.bitwise_and(mat,mat,finalMask, scaledThresh);

        Mat edges = new Mat();
        //detect edges of finalMask
        Imgproc.Canny(finalMask, edges, 100,200);

        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
        //RotatedRect because it allows for more accurate bounding rectangles, perfect if pole is slanted..
        RotatedRect[] rectangle = new RotatedRect[contours.size()];

        for(int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            //convert contour to approximate polygon..
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 1,true);
//            Imgproc.drawContours(input, contours, i, new Scalar(0,255,0), 1);
        }
        Imgproc.drawContours(input, contours, -1, new Scalar(0,255,0), 1);

/*
        //release all the data
//        input.release();

        Imgproc.cvtColor(scaledMask,input,Imgproc.COLOR_HSV2RGB);
        scaledThresh.copyTo(input);
        scaledThresh.release();
        scaledMask.release();
        mat.release();
        masked.release();
        edges.release();
        thresh.release();
        finalMask.release();
*/
//        thresh.copyTo(input);
        thresh.release();
        //finalMask.cvtColor(scaledMask,input,Imgproc.COLOR_HSV2RGB);
        finalMask.copyTo(input);
        edges.release();
        finalMask.release();
        scaledMask.release();
        scaledThresh.release();
        masked.release();
        mat.release();

        return input;
    }
}
