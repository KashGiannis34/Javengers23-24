package org.firstinspires.ftc.teamcode.ftc18305.auto.temp;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ftc18305.auto.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.ftc18305.test.camera.sample.BlueConePipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;


public class VisionAutoDrive {

    protected Servo leftTurret;
//    protected Servo rightTurret;

    public OpenCvWebcam webCamR;
    public OpenCvPipeline pipeline;

    //int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    int FIRST = 5;
    int SECOND = 10;
    int THIRD = 11;

    public static final int CAMERA_WIDTH = 800; //1280;
    public static final int CAMERA_HEIGHT = 448; //720;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;
    float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    int THRESHOLD_FRAMES_COUNT = 4;
    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    static final double FEET_PER_METER = 3.28084;

//    OpMode opMode;
//    RRAutoDrive() {
//        super();
//    }

//    RRAutoDrive(HardwareMap hwMap, OpMode _opMode) {
//        this(hwMap);
//        opMode = _opMode;
//    }

    public VisionAutoDrive(HardwareMap hwMap) {
//        super(hwMap);

        leftTurret = hwMap.get(Servo.class, "leftTurret");
//        rightTurret = hwMap.get(Servo.class, "rightTurret");

        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webCamR = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "WebcamR"), cameraMonitorViewId);
    }

    /**
     * Starts camera and sets camera to pipeline.
     */
    public void startCamera(OpenCvPipeline pipeline) {
        Log.d("StartCamera", "Creating Camera");
        if (webCamR != null) {
            Log.d("startCamera", "Creating Pipeline");
            pipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
            webCamR.setPipeline(pipeline);
            webCamR.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {

                    Log.d("StartCamera", "Creating Stream");
                    webCamR.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN);
                }

                @Override
                public void onError(int i) {

                }
            });
        }
        //FtcDashboard.getInstance().startCameraStream(webCamR, 10);
        Log.d("startCamera", "Camera Initialized");
    }

    public int getImagePosition() {
        int position = 0;
        int framesCount = 0;
        AprilTagDetectionPipeline aprilTagPipeline = (AprilTagDetectionPipeline) pipeline;
        ArrayList<AprilTagDetection> currentDetections = aprilTagPipeline.getLatestDetections();
        if(currentDetections != null) {
//            opMode.telemetry.addData("FPS", webCamR.getFps());
//            opMode.telemetry.addData("Overhead ms", webCamR.getOverheadTimeMs());
//            opMode.telemetry.addData("aprilTagPipeline ms", webCamR.getPipelineTimeMs());

            if (currentDetections.size() == 0) {
                framesCount++;
                if(framesCount >= THRESHOLD_FRAMES_COUNT) {
                    aprilTagPipeline.setDecimation(DECIMATION_LOW);
                }

            } else {
                framesCount = 0;

                if(currentDetections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                    aprilTagPipeline.setDecimation(DECIMATION_HIGH);
                }

                for (AprilTagDetection tag : currentDetections) {
//                    opMode.telemetry.addLine(String.format("\nDetected tag ID=%d", tag.id));
//                    opMode.telemetry.addLine(String.format("Translation X: %.2f feet", tag.pose.x*FEET_PER_METER));
//                    opMode.telemetry.addLine(String.format("Translation Y: %.2f feet", tag.pose.y*FEET_PER_METER));
//                    opMode.telemetry.addLine(String.format("Translation Z: %.2f feet", tag.pose.z*FEET_PER_METER));
//                    opMode.telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(tag.pose.yaw)));
//                    opMode.telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(tag.pose.pitch)));
//                    opMode.telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(tag.pose.roll)));

                    if (tag.id == FIRST || tag.id == SECOND || tag.id == THIRD) {
                        position = tag.id;
                        if (tag.id == FIRST) {
                            position = 1;
                        } else if (tag.id == SECOND) {
                            position = 2;
                        } else if (tag.id == THIRD) {
                            position = 3;
                        }
                        break;
                    }
                }
            }
//            opMode.telemetry.update();
        }
        return position;
    }

    public void switchCameraToPolePipeline() {
        BlueConePipeline conePipeline = new BlueConePipeline();
        pipeline = conePipeline;
        webCamR.setPipeline(conePipeline);
    }


    public boolean alignTurret(Telemetry telemetry) {
        boolean isAligned = false;
        BlueConePipeline conePipeline = (BlueConePipeline) pipeline;

        double centerX = conePipeline.getCenterX();
        double threshold = 250;
        ElapsedTime timer = new ElapsedTime();
//        while (!isAligned) {
        if(conePipeline.isBlueVisible()) {
            double imageCenterX = conePipeline.getCenterofRect(conePipeline.getBlueRect()).x;
            double diff = centerX - imageCenterX;
            if(Math.abs(diff) > threshold) {
                if(diff > 0) {
//                    turretServo.setPosition(RobotConstants.turretServo.getPosition() - 0.0005);
                    leftTurret.setPosition(Range.clip(leftTurret.getPosition() - .0005, 0, 1));
//                    rightTurret.setPosition(Range.clip(rightTurret.getPosition() + .0005, 0, 1));
                } else if(diff < 0) {
//                    turretServo.setPosition(RobotConstants.turretServo.getPosition() + 0.0005);
                    leftTurret.setPosition(Range.clip(leftTurret.getPosition() + .0005, 0, 1));
//                    rightTurret.setPosition(Range.clip(rightTurret.getPosition() - .0005, 0, 1));
                }
            } else {
                isAligned = true;
            }
            telemetry.addData("leftTurret position: ", leftTurret.getPosition());
//            telemetry.addData("rightTurret position: ", rightTurret.getPosition());
            telemetry.update();
        }
//        }
        return isAligned;
    }

}