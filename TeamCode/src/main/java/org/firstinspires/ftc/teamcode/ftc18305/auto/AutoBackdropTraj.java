package org.firstinspires.ftc.teamcode.ftc18305.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "TrajTest2Backdrop")
public class AutoBackdropTraj extends LinearOpMode {
    RRAutoDrive drive;
    String curAlliance = "red";

    private TeamElementSubsystem teamElementDetection;

    private int liftMotorTicks = 425;

    SplitAveragePipeline.ZONE zone = SplitAveragePipeline.ZONE.RIGHT;
    TrajectorySequence trajSeqCenter, trajSeqLeft, trajSeqRight;
    TrajectoryVelocityConstraint velCon = SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    TrajectoryAccelerationConstraint accCon = SampleMecanumDrive.getAccelerationConstraint(40);
    TrajectoryVelocityConstraint velConPixel = SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    TrajectoryAccelerationConstraint accConPixel = SampleMecanumDrive.getAccelerationConstraint(20);

    public void runOpMode()
    {
        initialize();

        teamElementDetection = new TeamElementSubsystem(hardwareMap, SplitAveragePipeline.ZONE_VIEW.LEFT);
        while (!isStarted() && !isStopRequested())
        {
            zone = teamElementDetection.getElementZoneValue(telemetry);
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

        if (curAlliance == "red") {
            Pose2d startPose = new Pose2d(18, -64, Math.toRadians(90));

            drive.setPoseEstimate(startPose);

            trajSeqCenter = drive.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(18, -30), velCon, accCon)
                    .lineToLinearHeading(new Pose2d(48, -36, Math.toRadians(180)), velCon, accCon)
                    .lineTo(new Vector2d(48, -64), velCon, accCon)
                    .build();
            trajSeqLeft = drive.trajectorySequenceBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(10, -31, Math.toRadians(180)), Math.toRadians(180), velConPixel, accConPixel)
                    .lineToLinearHeading(new Pose2d(48, -30, Math.toRadians(180)), velCon, accCon)
                    .lineTo(new Vector2d(48, -64), velCon, accCon)
                    .build();
            trajSeqRight = drive.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(25, -38), velCon, accCon)
//                    .lineTo(new Vector2d(36.5, -40), velCon, accCon)
                    .lineToSplineHeading(new Pose2d(48, -42, Math.toRadians(180)), velCon, accCon)
                    .lineTo(new Vector2d(48, -64), velCon, accCon)
                    .build();
        }
        else
        {
            Pose2d startPose = new Pose2d(18, 64, Math.toRadians(270));

            drive.setPoseEstimate(startPose);

            trajSeqCenter = drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(() ->
                    {
                        drive.restArm();
                    })
                    .lineTo(new Vector2d(18, 30), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(0, () ->
                    {
                        drive.activateLift(liftMotorTicks);
                        drive.activateArm();
                    })
                    .lineToLinearHeading(new Pose2d(54, 38.5, Math.toRadians(180)), velConPixel, accConPixel)
                    .waitSeconds(1)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1, () ->
                    {
                        drive.restArm();
                    })
                    .lineTo(new Vector2d(53, 38.5), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
                    .lineTo(new Vector2d(48, 62), velCon, accCon)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateLift();
                    })
                    .build();
            trajSeqRight = drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(() ->
                    {
                        drive.restArm();
                    })
                    .splineToLinearHeading(new Pose2d(8, 31, Math.toRadians(180)), Math.toRadians(180), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(0, () ->
                    {
                        drive.activateLift(liftMotorTicks);
                        drive.activateArm();
                    })
                    .lineToLinearHeading(new Pose2d(54.8, 31, Math.toRadians(180)), velCon, accCon)
                    .waitSeconds(1)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1, () ->
                    {
                        drive.restArm();
                    })
                    .lineTo(new Vector2d(53.8, 31), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
                    .lineTo(new Vector2d(48, 62), velCon, accCon)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateLift();
                    })
                    .build();
            trajSeqLeft = drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(() ->
                    {
                        drive.restArm();
                    })
                    .lineTo(new Vector2d(24, 38), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(0, () ->
                    {
                        drive.activateLift(liftMotorTicks);
                        drive.activateArm();
                    })
//                    .lineTo(new Vector2d(24+(54.8-24)/2, 38.5), velCon, accCon)
                    .lineToSplineHeading(new Pose2d(54.4, 39, Math.toRadians(180)), velConPixel, accConPixel)
                    .waitSeconds(1)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1, () ->
                    {
                        drive.restArm();
                    })
                    .lineTo(new Vector2d(53.4, 39), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
                    .lineTo(new Vector2d(48, 62), velCon, accCon)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateLift();
                    })
                    .build();
        }

        zone = SplitAveragePipeline.ZONE.LEFT;

        waitForStart();
        if (isStopRequested()) return;
        if (zone == SplitAveragePipeline.ZONE.LEFT)
            drive.followTrajectorySequence(trajSeqLeft);
        else if (zone == SplitAveragePipeline.ZONE.CENTER)
            drive.followTrajectorySequence(trajSeqCenter);
        else
            drive.followTrajectorySequence(trajSeqRight);
    }

    private void initialize() {
        drive = new RRAutoDrive(hardwareMap);
        drive.activateLeftClaw();
        drive.activateRightClaw();
        telemetry.addData("Status:", "initialize() - Robot and Camera are initialized!");
        telemetry.update();
    }
}
