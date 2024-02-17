package org.firstinspires.ftc.teamcode.ftc8468.auto.competitionOpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ftc8468.auto.RRAutoDrive;
import org.firstinspires.ftc.teamcode.ftc8468.auto.pipelines.TeamElementSubsystem;
import org.firstinspires.ftc.teamcode.ftc8468.auto.pipelines.SplitAveragePipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "AutoRedBackdropFARPARK")
public class AutoRedBackdropFARPARK extends LinearOpMode {
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

        teamElementDetection = new TeamElementSubsystem(hardwareMap, SplitAveragePipeline.ZONE_VIEW.RIGHT);
        while (!isStarted() && !isStopRequested())
        {
            zone = teamElementDetection.getElementZoneValue(telemetry);
//            if (gamepad1.x){
//                curAlliance = "blue";
//                teamElementDetection.setZoneView(SplitAveragePipeline.ZONE_VIEW.LEFT);
//            }else if (gamepad1.b){
//                curAlliance = "red";
//                teamElementDetection.setZoneView(SplitAveragePipeline.ZONE_VIEW.RIGHT);
//            }
            teamElementDetection.setAlliance(curAlliance);
//            telemetry.addData("Select Alliance (Gamepad1 X = Blue, Gamepad1 B = Red)", "");
            telemetry.addData("Current Alliance Selected : ", curAlliance.toUpperCase());
            telemetry.update();
        }

        if (curAlliance == "red") {
            Pose2d startPose = new Pose2d(18, -64, Math.toRadians(90));

            drive.setPoseEstimate(startPose);

            trajSeqCenter = drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(() ->
                    {
                        drive.initArm();
                        drive.deactivateIntakeServo();
                    })
                    .lineTo(new Vector2d(18, -30), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(0, () ->
                    {
                        drive.activateLift(liftMotorTicks);
                        drive.activateArm();
                    })
                    .lineToLinearHeading(new Pose2d(56.25, -36, Math.toRadians(180)), velConPixel, accConPixel)
                    .waitSeconds(1)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .lineTo(new Vector2d(52.25, -37), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
                    .UNSTABLE_addTemporalMarkerOffset(1, () ->
                    {
                        drive.restArm();
                    })
                    .lineTo(new Vector2d(48, -14), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateLift();
                        drive.activateIntakeServo();

                    })
                    .lineTo(new Vector2d(60, -14), velConPixel, accConPixel)
                    .build();
            trajSeqLeft = drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(() ->
                    {
                        drive.initArm();
                        drive.deactivateIntakeServo();
                    })
                    .splineToLinearHeading(new Pose2d(10, -31.5, Math.toRadians(180)), Math.toRadians(180), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(0, () ->
                    {
                        drive.activateLift(liftMotorTicks);
                        drive.activateArm();
                    })
                    .lineToLinearHeading(new Pose2d(55.75, -31.5, Math.toRadians(180)), velConPixel, accConPixel)
                    .waitSeconds(1)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .lineTo(new Vector2d(52.75, -31.5), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
                    .UNSTABLE_addTemporalMarkerOffset(1, () ->
                    {
                        drive.restArm();
                    })
                    .lineTo(new Vector2d(48, -14), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateLift();
                        drive.activateIntakeServo();
                    })
                    .lineTo(new Vector2d(60, -14), velConPixel, accConPixel)
                    .build();
            trajSeqRight = drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(() ->
                    {
                        drive.initArm();
                        drive.deactivateIntakeServo();
                    })
                    .lineTo(new Vector2d(25, -38), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(0, () ->
                    {
                        drive.activateLift(liftMotorTicks);
                        drive.activateArm();
                    })
                    .lineToSplineHeading(new Pose2d(55, -43, Math.toRadians(180)), velConPixel, accConPixel)
                    .waitSeconds(1)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .lineTo(new Vector2d(52, -43), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
                    .UNSTABLE_addTemporalMarkerOffset(1, () ->
                    {
                        drive.restArm();
                    })
                    .lineTo(new Vector2d(53, -14), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateLift();
                        drive.activateIntakeServo();
                    })
                    .lineTo(new Vector2d(60, -14), velConPixel, accConPixel)
                    .build();
        }
        else {
            // do nothing!
        }

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
        drive.deactivateIntakeServo();
        drive.initArm();
        telemetry.addData("Status:", "initialize() - Robot and Camera are initialized!");
        telemetry.update();
    }
}
