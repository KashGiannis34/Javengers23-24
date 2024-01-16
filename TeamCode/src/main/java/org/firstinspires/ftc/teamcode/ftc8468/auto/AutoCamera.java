package org.firstinspires.ftc.teamcode.ftc8468.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AutoCamera extends LinearOpMode {
    public int element_zone = 1;
    public SplitAveragePipeline.ZONE zoneValue = SplitAveragePipeline.ZONE.RIGHT;
    public SplitAveragePipeline.ZONE_VIEW zoneView = SplitAveragePipeline.ZONE_VIEW.LEFT;

    private TeamElementSubsystem teamElementDetection=null;

    boolean togglePreview = true;

    public void HardwareStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();

        teamElementDetection = new TeamElementSubsystem(hardwareMap, zoneView);

        telemetry.addData("Object Creation", "Done");
        telemetry.update();
    }



    public void runOpMode(){

        HardwareStart();

        String curAlliance = "red";

        while (!opModeIsActive() && !isStopRequested()){
//            element_zone = teamElementDetection.elementDetection(telemetry);
            zoneValue = teamElementDetection.getElementZoneValue(telemetry);
            telemetry.addData("getMaxDistance", teamElementDetection.getMaxDistance());

//            if (togglePreview && gamepad1.a){
//                togglePreview = false;
//                teamElementDetection.toggleAverageZone();
//            }else if (!gamepad2.a){
//                togglePreview = true;
//            }

            if (gamepad1.x){
                curAlliance = "blue";
            }else if (gamepad1.b){
                curAlliance = "red";
            }
            teamElementDetection.setAlliance(curAlliance);
            telemetry.addData("Select Alliance (Gamepad1 X = Blue, Gamepad1 B = Red)", "");
            telemetry.addData("Current Alliance Selected : ", curAlliance.toUpperCase());

            telemetry.update();
        }

        telemetry.addData("Object", "Passed waitForStart");

        telemetry.update();

    }
}
