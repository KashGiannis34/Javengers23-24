package org.firstinspires.ftc.teamcode.ftc8468.auto.pipelines.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ftc8468.auto.pipelines.ContourPipeline;
import org.firstinspires.ftc.teamcode.ftc8468.auto.pipelines.DetectWhiteStack;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class AutoCameraPixelStack extends LinearOpMode {
    OpenCvCamera camera;
    private ContourPipeline pixelStackPipeline;
    int camW = 1280;
    int camH = 720;

    boolean togglePreview = true;

    public void HardwareStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();

        pixelStackPipeline = new ContourPipeline(0,0,0,0);

        telemetry.addData("Object Creation", "Done");
        telemetry.update();

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        camera.setPipeline(pixelStackPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(camW, camH, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }



    public void runOpMode(){

        HardwareStart();

        String curAlliance = "red";

        while (!opModeIsActive() && !isStopRequested()){
//            element_zone = teamElementDetection.elementDetection(telemetry);
            telemetry.addData("Center: ", pixelStackPipeline.getRectMidpointXY());
            telemetry.addData("Height: ", pixelStackPipeline.getRectArea());

            telemetry.update();
        }

        telemetry.addData("Object", "Passed waitForStart");

        telemetry.update();

    }
}
