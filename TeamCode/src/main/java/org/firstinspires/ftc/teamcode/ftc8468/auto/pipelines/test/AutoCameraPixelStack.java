package org.firstinspires.ftc.teamcode.ftc8468.auto.pipelines.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ftc8468.auto.pipelines.PixelStackPipeline;
import org.firstinspires.ftc.teamcode.ftc8468.auto.pipelines.SplitAveragePipeline;
import org.firstinspires.ftc.teamcode.ftc8468.auto.pipelines.TeamElementSubsystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class AutoCameraPixelStack extends LinearOpMode {
    OpenCvCamera camera;
    private PixelStackPipeline pixelStackPipeline;
    int camW = 1280;
    int camH = 720;

    boolean togglePreview = true;

    public void HardwareStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();

        pixelStackPipeline = new PixelStackPipeline(camW);

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
            telemetry.addData("StackLocation: ", pixelStackPipeline.getLocation());
            telemetry.addData("ObjectWidthCenter: ", pixelStackPipeline.getObjectWidthCenter());

            telemetry.update();
        }

        telemetry.addData("Object", "Passed waitForStart");

        telemetry.update();

    }
}
