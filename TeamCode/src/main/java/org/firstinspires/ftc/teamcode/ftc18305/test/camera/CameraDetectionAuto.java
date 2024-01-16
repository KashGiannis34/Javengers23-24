package org.firstinspires.ftc.teamcode.ftc18305.test.camera;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


//@Autonomous
public class CameraDetectionAuto extends LinearOpMode {
    RobotDriveTemp drive;
    private static int elementPosition = 0;

    Trajectory trajLow1, trajLow2, trajLow3, trajLow4, trajLow5, trajLow6, trajLow7, trajLow8, trajLow9, trajLow10, trajLow11, trajLow12, trajLow13, trajLow14;
    Trajectory trajMid1, trajMid2, trajMid3, trajMid4, trajMid5, trajMid6, trajMid7, trajMid8, trajMid9, trajMid10, trajMid11, trajMid12, trajMid13, trajMid14;
    Trajectory trajHigh1, trajHigh2, trajHigh3, trajHigh4, trajHigh5, trajHigh6, trajHigh7, trajHigh8, trajHigh9, trajHigh10, trajHigh11, trajHigh12, trajHigh13, trajHigh14;


    // RPM = 1620; TICKS_PER_REV = 103.8; Gear Ratio = 5;
    int TICKS_FOR_ONE = 519; // Ticks for 1 rotation is 103.8 * 5 = 58;
    int TICKS_FOR_TWO = 1038; // Ticks for 2 rotation is 28 * 10 = 1538;
    int TICKS_FOR_THREE = 1557; // Ticks for 3 rotation is 28 * 15 = 2307;
    private int liftMotorTicks = TICKS_FOR_THREE;


    public void runOpMode() {
        initialize();

        waitForStart();
        if (isStopRequested()) return;

//        elementPosition = JunctionObserverPipeline.position;
//        telemetry.addData("Position: ", elementPosition);
//        telemetry.update();
//
//        if(elementPosition == 1) {
//            drive.followTrajectoryAsync(trajLow1);
//        } else if(elementPosition == 2) {
//            drive.followTrajectoryAsync(trajMid1);
//        } else {
//            //position = 3
//            drive.followTrajectoryAsync(trajHigh1);
//        }

        while (opModeIsActive() && !isStopRequested()) {
//            drive.update();
//            drive.checkLiftMotion();
        }

    }

    private void initialize() {
        drive = new RobotDriveTemp(hardwareMap);
        drive.startCamera();
//        JunctionObserverPipeline.setRegionValues(6, 72, 5);
        telemetry.addData("Status:", "initialize() - Robot and Camera are initialized!");
        telemetry.update();
    }

}
