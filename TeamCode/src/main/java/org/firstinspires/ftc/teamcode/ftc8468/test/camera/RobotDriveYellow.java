package org.firstinspires.ftc.teamcode.ftc8468.test.camera;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ftc8468.test.camera.sample.YellowPolePipelineOld;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class RobotDriveYellow {

    public static final int CAMERA_WIDTH = 1280; //320;
    public static final int CAMERA_HEIGHT = 720; //240;

    protected Servo turretServo;
    public OpenCvWebcam webCamR;
    public YellowPolePipelineOld pipeline;

    public RobotDriveYellow(HardwareMap hwMap) {
//        super(hwMap);
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        if (webCamR == null) {
            webCamR = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "WebcamL"), cameraMonitorViewId);
            webCamR.setMillisecondsPermissionTimeout(50000);
        }

        turretServo = hwMap.get(Servo.class, "turretServo");
    }

    /**
     * Starts camera and sets camera to pipeline.
     */
    public void startCamera() {
        Log.d("StartCamera", "Creating Camera");
        if (webCamR != null) {
            webCamR.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {

                    Log.d("StartCamera", "Creating Stream");
                    webCamR.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);


                    Log.d("startCamera", "Creating Pipeline");
                    pipeline = new YellowPolePipelineOld();
                    webCamR.setPipeline(pipeline);
                }

                @Override
                public void onError(int i) {

                }
            });
        }
        //FtcDashboard.getInstance().startCameraStream(webCamR, 10);
        Log.d("startCamera", "Camera Initialized");
    }

    /**
     * Starts camera and sets camera to pipeline.
     */
    public void startCamera(OpenCvPipeline pipeline) {
        Log.d("StartCamera", "Creating Camera");
        if (webCamR != null) {
            webCamR.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {

                    Log.d("StartCamera", "Creating Stream");
                    webCamR.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);


                    Log.d("startCamera", "Creating Pipeline");
                    webCamR.setPipeline(pipeline);
                }

                @Override
                public void onError(int i) {


                }
            });
        }
        //FtcDashboard.getInstance().startCameraStream(webCamR, 10);
        Log.d("startCamera", "Camera Initialized");
    }

}
