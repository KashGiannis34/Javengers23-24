package org.firstinspires.ftc.teamcode.ftc18305.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous
public class AutoBlueTerminal extends LinearOpMode {
    // "blue" for Blue and "red" for Red Alliance
    String ALLIANCE = "blue";
    public int element_zone = 1;

    private String element_position = "";
    public String zoneValue = "RIGHT";
    private TeamElementSubsystem teamElementDetection = null;

    RRAutoDrive drive;

    Trajectory traj1;

    public static final int CAMERA_WIDTH = 800; //1280;
    public static final int CAMERA_HEIGHT = 448; //720;

    protected float speedMultiplier = 0.6f;

    final int LIFT_TOLERANCE = 10;
    final int VERT_TOLERANCE = 0;
    final int HORIZ_TOLERANCE = 0;

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
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    static final double FEET_PER_METER = 3.28084;
    int numFramesWithoutDetection = 0;

    //TrajectorySequence trajSeqLow, trajSeqMid, trajSeqHigh;

    // RPM = 1620; TICKS_PER_REV = 103.8; Gear Ratio = 5;

    int TICKS_FOR_HIGH = 725; // Ticks for 3 rotation is 28 * 15 = 2307;
    private int vertLiftTicks = TICKS_FOR_HIGH;




    public void runOpMode() {
//        initialize();

        Pose2d startPose = new Pose2d();
        Pose2d nextPose = startPose;
        Pose2d nextPoseCycleAuto;

        drive.setPoseEstimate(startPose);
// Low Starts Here...
        nextPoseCycleAuto = startPose;
        traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-50,0))
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajLow2))
                .build();
        nextPoseCycleAuto = traj1.end();

        waitForStart();
        if (isStopRequested()) return;

        //drive.followTrajectorySequenceAsync(trajSeqHigh);

        // Check the element position after START!
//        element_zone = teamElementDetection.elementDetection(telemetry);
//        zoneValue = teamElementDetection.getElementZoneValue(telemetry);
        telemetry.addData("getMaxDistance", teamElementDetection.getMaxDistance());
        telemetry.update();

        drive.followTrajectory(traj1);

         if(zoneValue == "RIGHT") {
         //drive.followTrajectorySequenceAsync(trajSeqLow);
         } else if(zoneValue == "MIDDLE") {
         //drive.followTrajectorySequenceAsync(trajSeqMid);
         } else { // "LEFT"
         //drive.followTrajectorySequenceAsync(trajSeqHigh);
         }

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
//            // need to align the Intake Turret only while extending the Horizontal Slide..
//            isAligned = drive.alignIntakeTurret(telemetry);
//
//            drive.checkLiftMotion();

//            zoneValue = teamElementDetection.getElementZoneValue(telemetry);
//            telemetry.addData("getMaxDistance", teamElementDetection.getMaxDistance());
            telemetry.update();
        }

    }

//    private void initialize() {
//        drive = new RRAutoDrive(hardwareMap);
////        drive.startCamera();
//        teamElementDetection = new TeamElementSubsystem(hardwareMap, , ALLIANCE);
//        telemetry.addData("Status:", "initialize() - Robot and Camera are initialized!");
//        telemetry.update();
//    }


}
