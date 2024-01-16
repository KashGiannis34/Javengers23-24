package org.firstinspires.ftc.teamcode.ftc18305.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class TeamElementSubsystem {
    OpenCvCamera camera;
    SplitAveragePipeline splitAveragePipeline;
    // default is "red". Use the constructor to pass different alliance color
    int camW = 1280;
    int camH = 720;

    SplitAveragePipeline.ZONE zone = SplitAveragePipeline.ZONE.RIGHT; //
    String zoneValue = "LEFT";



    public TeamElementSubsystem(HardwareMap hardwareMap, SplitAveragePipeline.ZONE_VIEW zoneView){
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        splitAveragePipeline = new SplitAveragePipeline(zoneView);

        camera.setPipeline(splitAveragePipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(camW, camH, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    public TeamElementSubsystem(HardwareMap hardwareMap, SplitAveragePipeline.ZONE_VIEW zoneView, String alliance){
        this(hardwareMap, zoneView);
        setAlliance(alliance);
    }

    public void setAlliance(String alliance){
        splitAveragePipeline.setAlliancePipe(alliance);
    }

    public SplitAveragePipeline.ZONE elementDetection(Telemetry telemetry) {
        zone = splitAveragePipeline.get_element_zone();
        telemetry.addData("Element Zone", zone);
        return zone;
    }

    public SplitAveragePipeline.ZONE getElementZoneValue(Telemetry telemetry){
        String value = "";
        zone = splitAveragePipeline.get_element_zone();

        if (zone == SplitAveragePipeline.ZONE.RIGHT) {
            value = "RIGHT";
        } else if (zone == SplitAveragePipeline.ZONE.CENTER) {
            value = "MIDDLE";
        } else if (zone == SplitAveragePipeline.ZONE.LEFT) {
            value = "LEFT";
        }
        telemetry.addData("Element Zone: ", zone + "Value: "+value);
        return zone;
    }

    public void toggleAverageZone(){
        splitAveragePipeline.toggleAverageZonePipe();
    }

    public double getMaxDistance(){
        return splitAveragePipeline.getMaxDistance();
    }
}
