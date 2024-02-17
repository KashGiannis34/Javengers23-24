package org.firstinspires.ftc.teamcode.ftc8468.auto.competitionOpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ftc8468.auto.RRAutoDrive;
import org.firstinspires.ftc.teamcode.ftc8468.auto.pipelines.TeamElementSubsystem;
import org.firstinspires.ftc.teamcode.ftc8468.auto.pipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.ftc8468.auto.pipelines.SplitAveragePipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.ArrayList;
@Disabled
@Autonomous (name = "TrajTest2HumanPlayer")
public class AutoHumanPlayerTraj extends LinearOpMode {
    RRAutoDrive drive;
    String curAlliance = "red";
    OpenCvCamera bCamera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

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

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    int TARGET_ID = 1;
    boolean gamepadXisPressed = false;
    boolean gamepadBisPressed = false;
    int action = -1;
    int index = 0;
    Vector2d finalVectorBlue = new Vector2d(44, 40);
    Vector2d finalVectorRed = new Vector2d(44, -40);

    double headingPower = 0;

    private TeamElementSubsystem teamElementDetection;

    private int liftMotorTicks = 425;

    SplitAveragePipeline.ZONE zone = SplitAveragePipeline.ZONE.RIGHT;
    TrajectorySequence trajSeqCenter, trajSeqLeft, trajSeqRight, trajSeqRedFinal, trajSeqBlueFinal;
    TrajectoryVelocityConstraint velCon = SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    TrajectoryAccelerationConstraint accCon = SampleMecanumDrive.getAccelerationConstraint(40);
    TrajectoryVelocityConstraint velConPixel = SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    TrajectoryAccelerationConstraint accConPixel = SampleMecanumDrive.getAccelerationConstraint(15);

    public void runOpMode()
    {
        initialize();

        teamElementDetection = new TeamElementSubsystem(hardwareMap, SplitAveragePipeline.ZONE_VIEW.LEFT);
        while (!isStarted() && !isStopRequested())
        {
            zone = teamElementDetection.getElementZoneValue(telemetry);
            if (gamepad1.x){
                curAlliance = "blue";
                teamElementDetection.setZoneView(SplitAveragePipeline.ZONE_VIEW.RIGHT);
            }else if (gamepad1.b){
                curAlliance = "red";
                teamElementDetection.setZoneView(SplitAveragePipeline.ZONE_VIEW.LEFT);
            }
            teamElementDetection.setAlliance(curAlliance);
            telemetry.addData("Select Alliance (Gamepad1 X = Blue, Gamepad1 B = Red)", "");
            telemetry.addData("Current Alliance Selected : ", curAlliance.toUpperCase());
            telemetry.update();
        }

        if (curAlliance == "red") {
            Pose2d startPose = new Pose2d(-42, -64, Math.toRadians(90));

            drive.setPoseEstimate(startPose);

            trajSeqCenter = drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(() ->
                    {
                        drive.restArm();
                    })
                    // spline traj
//                    .lineTo(new Vector2d(-39, -30), velConPixel, accConPixel) // drive to team prop
//                    .lineTo(new Vector2d(-39, -36), velConPixel, accConPixel)
//                    .splineToConstantHeading(new Vector2d(-42, -42), Math.toRadians(180), velConPixel, accConPixel)
//                    .lineTo(new Vector2d(-52, -42), velConPixel, accConPixel)
//                    .splineToConstantHeading(new Vector2d(-56, -36), Math.toRadians(90), velConPixel, accConPixel)
//                    .lineTo(new Vector2d(-56, -16), velConPixel, accConPixel)
//                    .splineToConstantHeading(new Vector2d(-52, -10), Math.toRadians(0), velConPixel, accConPixel)
//                    .lineToSplineHeading(new Pose2d(-24, -10, Math.toRadians(180)), velConPixel, accConPixel)
//                    .lineTo(new Vector2d(30, -10), velConPixel, accConPixel)
                    .lineToConstantHeading(new Vector2d(-39, -30), velConPixel, accConPixel) // no-spline traj
                    .lineToConstantHeading(new Vector2d(-39, -42), velConPixel, accConPixel)
                    .lineToConstantHeading(new Vector2d(-56, -42), velConPixel, accConPixel)
                    .lineToConstantHeading(new Vector2d(-56, -10), velConPixel, accConPixel)
                    .lineToSplineHeading(new Pose2d(-24, -10, Math.toRadians(180)), velConPixel, accConPixel)
                    .lineToConstantHeading(new Vector2d(22, -10), velConPixel,accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(24, () ->
                    {
                        drive.activateLift(liftMotorTicks);
                        drive.activateArm();
                    })
                    .lineToConstantHeading(finalVectorRed, velConPixel, accConPixel)
////                    .splineToConstantHeading(new Vector2d(51.5, -35.5), Math.toRadians(270), velConPixel, accConPixel) //removed spline traj
//                    .waitSeconds(1)
//                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
//                        drive.deactivateRightClaw();
//                        drive.deactivateLeftClaw();
//                    })
//                    .UNSTABLE_addTemporalMarkerOffset(1, () ->
//                    {
//                        drive.restArm();
//                    })
//                    .lineToConstantHeading(new Vector2d(49.5, -36), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
//                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
//                        drive.deactivateLift();
//                    })
                    .build();
            trajSeqLeft = drive.trajectorySequenceBuilder(startPose)
                    // spline path
//                    .lineToLinearHeading(new Pose2d(-42, -34, Math.toRadians(120)), velConPixel, accConPixel)
//                    .lineTo(new Vector2d(-36, -34), velConPixel, accConPixel)
//                    .lineToSplineHeading(new Pose2d(-32, -34, Math.toRadians(180)), velConPixel, accConPixel)
//                    .splineToConstantHeading(new Vector2d(-29, -24), Math.toRadians(90), velConPixel, accConPixel)
//                    .lineTo(new Vector2d(-29, -16), velConPixel, accConPixel)
//                    .splineToConstantHeading(new Vector2d(-20, -8), Math.toRadians(0), velConPixel, accConPixel)
//                    .lineTo(new Vector2d(30, -8), velCon, accCon)
//                    .splineToConstantHeading(new Vector2d(52, -30), Math.toRadians(270), velConPixel, accConPixel)
                    .addTemporalMarker(() ->
                    {
                        drive.restArm();
                    })
                    .lineToLinearHeading(new Pose2d(-42, -34, Math.toRadians(120)), velConPixel, accConPixel)
//                    .lineToConstantHeading(new Vector2d(-36, -34), velConPixel, accConPixel)
                    .lineToSplineHeading(new Pose2d(-35.5, -34, Math.toRadians(180)), velConPixel, accConPixel)
                    .lineToConstantHeading(new Vector2d(-32, -10), velConPixel, accConPixel)
                    .lineToConstantHeading(new Vector2d(22, -10), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(24, () ->
                    {
                        drive.activateLift(liftMotorTicks);
                        drive.activateArm();
                    })
                    .lineToConstantHeading(finalVectorRed, velConPixel, accConPixel)
//                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
//                        drive.deactivateRightClaw();
//                        drive.deactivateLeftClaw();
//                    })
//                    .UNSTABLE_addTemporalMarkerOffset(1, () ->
//                    {
//                        drive.restArm();
//                    })
//                    .lineToConstantHeading(new Vector2d(49.5, -36), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
//                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
//                        drive.deactivateLift();
//                    })
                    .build();
            trajSeqRight = drive.trajectorySequenceBuilder(startPose)
                    // spline path
//                    .splineToLinearHeading(new Pose2d(-32, -34, Math.toRadians(0)), Math.toRadians(0), velConPixel, accConPixel)
//                    .lineTo(new Vector2d(-36, -34), velConPixel, accConPixel)
//                    .splineToConstantHeading(new Vector2d(-38, -32), Math.toRadians(90), velConPixel, accConPixel)
//                    .lineTo(new Vector2d(-38, -16), velConPixel, accConPixel)
//                    .splineToConstantHeading(new Vector2d(-34, -8), Math.toRadians(0), velConPixel, accConPixel)
//                    .lineTo(new Vector2d(30, -8), velConPixel, accConPixel)
//                    .splineToConstantHeading(new Vector2d(52, -42), Math.toRadians(270), velConPixel, accConPixel)
                    .addTemporalMarker(() ->
                    {
                        drive.restArm();
                    })
                    .splineToLinearHeading(new Pose2d(-32, -38, Math.toRadians(20)), Math.toRadians(0), velConPixel, accConPixel)
                    .lineToLinearHeading(new Pose2d(-40, -34, Math.toRadians(0)), velConPixel, accConPixel)
                    .lineToLinearHeading(new Pose2d(-40, -10, Math.toRadians(180)), velConPixel, accConPixel)
                    .lineToConstantHeading(new Vector2d(22, -10), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(24, () ->
                    {
                        drive.activateLift(liftMotorTicks);
                        drive.activateArm();
                    })
                    .lineToConstantHeading(finalVectorRed, velConPixel, accConPixel)
//                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
//                        drive.deactivateRightClaw();
//                        drive.deactivateLeftClaw();
//                    })
//                    .UNSTABLE_addTemporalMarkerOffset(1, () ->
//                    {
//                        drive.restArm();
//                    })
//                    .lineToConstantHeading(new Vector2d(49.5, -36), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
//                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
//                        drive.deactivateLift();
//                    })
                    .build();
        }
        else
        {
            Pose2d startPose = new Pose2d(-42, 64, Math.toRadians(270));

            drive.setPoseEstimate(startPose);

            trajSeqCenter = drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(() ->
                    {
                        drive.restArm();
                    })
                    // spline traj
//                    .lineTo(new Vector2d(-39, -30), velConPixel, accConPixel) // drive to team prop
//                    .lineTo(new Vector2d(-39, -36), velConPixel, accConPixel)
//                    .splineToConstantHeading(new Vector2d(-42, -42), Math.toRadians(180), velConPixel, accConPixel)
//                    .lineTo(new Vector2d(-52, -42), velConPixel, accConPixel)
//                    .splineToConstantHeading(new Vector2d(-56, -36), Math.toRadians(90), velConPixel, accConPixel)
//                    .lineTo(new Vector2d(-56, -16), velConPixel, accConPixel)
//                    .splineToConstantHeading(new Vector2d(-52, -10), Math.toRadians(0), velConPixel, accConPixel)
//                    .lineToSplineHeading(new Pose2d(-24, -10, Math.toRadians(180)), velConPixel, accConPixel)
//                    .lineTo(new Vector2d(30, -10), velConPixel, accConPixel)
                    .lineToConstantHeading(new Vector2d(-39, 30), velConPixel, accConPixel) // no-spline traj
                    .lineToConstantHeading(new Vector2d(-39, 42), velConPixel, accConPixel)
                    .lineToConstantHeading(new Vector2d(-56, 42), velConPixel, accConPixel)
                    .lineToConstantHeading(new Vector2d(-56, 10), velConPixel, accConPixel)
                    .lineToSplineHeading(new Pose2d(-24, 10, Math.toRadians(180)), velConPixel, accConPixel)
                    .lineToConstantHeading(new Vector2d(22, 10), velConPixel,accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(24, () ->
                    {
                        drive.activateLift(liftMotorTicks);
                        drive.activateArm();
                    })
                    .lineToConstantHeading(finalVectorBlue, velConPixel, accConPixel)
////                    .splineToConstantHeading(new Vector2d(51.5, -35.5), Math.toRadians(270), velConPixel, accConPixel) //removed spline traj
//                    .waitSeconds(1)
//                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
//                        drive.deactivateRightClaw();
//                        drive.deactivateLeftClaw();
//                    })
//                    .UNSTABLE_addTemporalMarkerOffset(1, () ->
//                    {
//                        drive.restArm();
//                    })
//                    .lineToConstantHeading(new Vector2d(49.5, 36), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
//                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
//                        drive.deactivateLift();
//                    })
                    .build();
            trajSeqRight = drive.trajectorySequenceBuilder(startPose)
                    // spline path
//                    .lineToLinearHeading(new Pose2d(-42, -34, Math.toRadians(120)), velConPixel, accConPixel)
//                    .lineTo(new Vector2d(-36, -34), velConPixel, accConPixel)
//                    .lineToSplineHeading(new Pose2d(-32, -34, Math.toRadians(180)), velConPixel, accConPixel)
//                    .splineToConstantHeading(new Vector2d(-29, -24), Math.toRadians(90), velConPixel, accConPixel)
//                    .lineTo(new Vector2d(-29, -16), velConPixel, accConPixel)
//                    .splineToConstantHeading(new Vector2d(-20, -8), Math.toRadians(0), velConPixel, accConPixel)
//                    .lineTo(new Vector2d(30, -8), velCon, accCon)
//                    .splineToConstantHeading(new Vector2d(52, -30), Math.toRadians(270), velConPixel, accConPixel)
                    .addTemporalMarker(() ->
                    {
                        drive.restArm();
                    })
                    .lineToLinearHeading(new Pose2d(-42, 34, Math.toRadians(240)), velConPixel, accConPixel)
//                    .lineToConstantHeading(new Vector2d(-36, -34), velConPixel, accConPixel)
                    .lineToSplineHeading(new Pose2d(-35.5, 34, Math.toRadians(180)), velConPixel, accConPixel)
                    .lineToConstantHeading(new Vector2d(-32, 10), velConPixel, accConPixel)
                    .lineToConstantHeading(new Vector2d(22, 10), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(24, () ->
                    {
                        drive.activateLift(liftMotorTicks);
                        drive.activateArm();
                    })
                    .lineToConstantHeading(finalVectorBlue, velConPixel, accConPixel)
//                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
//                        drive.deactivateRightClaw();
//                        drive.deactivateLeftClaw();
//                    })
//                    .UNSTABLE_addTemporalMarkerOffset(1, () ->
//                    {
//                        drive.restArm();
//                    })
//                    .lineToConstantHeading(new Vector2d(49.5, 36), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
//                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
//                        drive.deactivateLift();
//                    })
                    .build();
            trajSeqLeft = drive.trajectorySequenceBuilder(startPose)
                    // spline path
//                    .splineToLinearHeading(new Pose2d(-32, -34, Math.toRadians(0)), Math.toRadians(0), velConPixel, accConPixel)
//                    .lineTo(new Vector2d(-36, -34), velConPixel, accConPixel)
//                    .splineToConstantHeading(new Vector2d(-38, -32), Math.toRadians(90), velConPixel, accConPixel)
//                    .lineTo(new Vector2d(-38, -16), velConPixel, accConPixel)
//                    .splineToConstantHeading(new Vector2d(-34, -8), Math.toRadians(0), velConPixel, accConPixel)
//                    .lineTo(new Vector2d(30, -8), velConPixel, accConPixel)
//                    .splineToConstantHeading(new Vector2d(52, -42), Math.toRadians(270), velConPixel, accConPixel)
                    .addTemporalMarker(() ->
                    {
                        drive.restArm();
                    })
                    .splineToLinearHeading(new Pose2d(-32, 38, Math.toRadians(340)), Math.toRadians(0), velConPixel, accConPixel)
                    .lineToLinearHeading(new Pose2d(-40, 34, Math.toRadians(0)), velConPixel, accConPixel)
                    .lineToLinearHeading(new Pose2d(-40, 10, Math.toRadians(180)), velConPixel, accConPixel)
                    .lineToConstantHeading(new Vector2d(22, 10), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(24, () ->
                    {
                        drive.activateLift(liftMotorTicks);
                        drive.activateArm();
                    })
                    .lineToConstantHeading(finalVectorBlue, velConPixel, accConPixel)
//                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
//                        drive.deactivateRightClaw();
//                        drive.deactivateLeftClaw();
//                    })
//                    .UNSTABLE_addTemporalMarkerOffset(1, () ->
//                    {
//                        drive.restArm();
//                    })
//                    .lineToConstantHeading(new Vector2d(49.5, 36), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
//                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
//                        drive.deactivateLift();
//                    })
                    .build();
        }

        if (curAlliance == "red")
        {
            Pose2d startPose = new Pose2d(47, -40, Math.toRadians(180));
            trajSeqRedFinal = drive.trajectorySequenceBuilder(startPose)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1, () ->
                    {
                        drive.restArm();
                    })
                    .lineToConstantHeading(new Vector2d(51, -40), velConPixel, accConPixel)
                    .lineToConstantHeading(new Vector2d(startPose.getX()-2, -36), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateLift();
                    })
                    .build();
        }
        else
        {
            Pose2d startPose = new Pose2d(47, 36, Math.toRadians(180));
            trajSeqBlueFinal = drive.trajectorySequenceBuilder(startPose)
                    .lineToConstantHeading(new Vector2d(51, 40), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1, () ->
                    {
                        drive.restArm();
                    })
                    .lineToConstantHeading(new Vector2d(startPose.getX()-2, 40), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateLift();
                    })
                    .build();
        }

        waitForStart();
        if (isStopRequested()) return;
        if (zone == SplitAveragePipeline.ZONE.LEFT) {
            drive.followTrajectorySequence(trajSeqLeft);
            TARGET_ID = 1;
        }
        else if (zone == SplitAveragePipeline.ZONE.CENTER) {
            drive.followTrajectorySequence(trajSeqCenter);
            TARGET_ID = 2;
        }
        else {
            drive.followTrajectorySequence(trajSeqRight);
            TARGET_ID = 3;
        }

        bCamera.setPipeline(aprilTagDetectionPipeline);
        double heading;
        double rawHeading;
        boolean stopTraj = false;
        while (opModeIsActive() && !stopTraj)
        {
            telemetry.addData("Pose: ", drive.getPoseEstimate());
            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
            rawHeading = drive.getPoseEstimate().getHeading();
            heading = rawHeading - Math.PI;

            if (Math.abs(heading) > 0.04) {
                if (heading < 0)
                    headingPower = Range.clip(Math.abs(6 * (heading / (2 * Math.PI))) + Math.abs(9*Math.pow((heading / (2 * Math.PI)),2)), 0.15, 0.5);
                else
                    headingPower = -1 * Range.clip(Math.abs(6 * (heading / (2 * Math.PI))) + Math.abs(9*Math.pow((heading / (2 * Math.PI)), 2)), 0.15, 0.5);
            }
            else
                headingPower = 0;


            // If there's been a new frame...
            if(detections != null)
            {
                telemetry.addData("FPS", bCamera.getFps());
                telemetry.addData("Overhead ms", bCamera.getOverheadTimeMs());
                telemetry.addData("Pipeline ms", bCamera.getPipelineTimeMs());
                telemetry.addData("Robot pose: ", drive.getPoseEstimate());
                telemetry.addData("Action: ", action);
                telemetry.addData("Target ID: ", TARGET_ID);
                telemetry.addData("Heading Power: ", headingPower);
                telemetry.addData("Heading: ", heading);
                telemetry.addData("Raw Heading: ", rawHeading);

                // If we don't see any tags
                if(detections.size() == 0)
                {
                    drive.setWeightedDrivePower(new Pose2d(0,0,headingPower));
                }
                // We do see tags!
                else
                {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }

                    int curIndex = -1;
                    for(AprilTagDetection detection : detections)
                    {
                        curIndex++;
                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//                        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//                        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//                        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
                        if (detection.id == TARGET_ID) {
                            action = 0;
                            index = curIndex;
                        }
                        if (detection.id < TARGET_ID && action != 0)
                            action = 1;
                        if (detection.id > TARGET_ID && action != 0)
                            action = -1;
                    }

                    if (action == 0) {
                        if (index < detections.size()) {
                            if (detections.get(index).pose.x > .1)
                                drive.setWeightedDrivePower(new Pose2d(0, 0.4, headingPower));
                            else if (detections.get(index).pose.x < -.1)
                                drive.setWeightedDrivePower(new Pose2d(0, -0.4, headingPower));
                            else {
                                drive.setWeightedDrivePower(new Pose2d(0, 0, headingPower));
                                stopTraj = true;
                            }
                        }
                        else {
                            drive.setWeightedDrivePower(new Pose2d(0, 0, headingPower));
                        }
                    }
                    else if (action == -1)
                    {
                        drive.setWeightedDrivePower(new Pose2d(0,-0.4, headingPower));
                    }
                    else
                    {
                        drive.setWeightedDrivePower(new Pose2d(0,0.4, headingPower));
                    }
                }
                drive.update();
            }
            telemetry.update();
        }
        if (curAlliance == "red")
            drive.followTrajectorySequence(trajSeqRedFinal);
        else
            drive.followTrajectorySequence(trajSeqBlueFinal);
    }

    private void initialize() {
        drive = new RRAutoDrive(hardwareMap);
        drive.activateLeftClaw();
        drive.activateRightClaw();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        bCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "WebcamR"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        telemetry.addData("Status:", "initialize() - Robot and Camera are initialized!");
        telemetry.update();
    }
}
