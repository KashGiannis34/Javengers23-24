package org.firstinspires.ftc.teamcode.ftc8468.auto.competitionOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ftc8468.auto.RRAutoDrive;
import org.firstinspires.ftc.teamcode.ftc8468.auto.pipelines.SplitAveragePipeline;
import org.firstinspires.ftc.teamcode.ftc8468.auto.pipelines.TeamElementSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous (name = "AutoBlueBackdropStackOnceCycle")
public class AutoBlueBackdropStack extends LinearOpMode {
    RRAutoDrive drive;
    String curAlliance = "blue";

    private TeamElementSubsystem teamElementDetection;

    private int liftMotorTicks = 390;

    SplitAveragePipeline.ZONE zone = SplitAveragePipeline.ZONE.RIGHT;
    public TrajectorySequence trajSeqCenter, trajSeqLeft, trajSeqRight, trajSeqStackDrive;
    TrajectoryVelocityConstraint velConPixel = SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    TrajectoryAccelerationConstraint accConPixel = SampleMecanumDrive.getAccelerationConstraint(30);

    TrajectoryVelocityConstraint velConBackdrop = SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    TrajectoryAccelerationConstraint accConBackdrop = SampleMecanumDrive.getAccelerationConstraint(20);

    public void runOpMode()
    {
        initialize();

        teamElementDetection = new TeamElementSubsystem(hardwareMap, SplitAveragePipeline.ZONE_VIEW.LEFT);
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

        if (curAlliance == "blue") {
            Pose2d startPose = new Pose2d(18, 64, Math.toRadians(270));

            drive.setPoseEstimate(startPose);

            trajSeqCenter = drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(() ->
                    {
                        drive.initArm();
                        drive.deactivateIntakeServo();
                        drive.activateLift(liftMotorTicks);
                    })
                    .lineTo(new Vector2d(18, 32), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(0, () ->
                    {
                        drive.activateArm();
                    })
                    .lineToLinearHeading(new Pose2d(54.75, 32, Math.toRadians(180)), velConBackdrop, accConBackdrop)
                    .waitSeconds(1)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .lineTo(new Vector2d(51.75, 32), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
                    .UNSTABLE_addDisplacementMarkerOffset(0, () ->
                    {
                        drive.restArmAuto();
                        drive.deactivateLift();
                    })
                    .UNSTABLE_addDisplacementMarkerOffset(15, () ->
                    {
                        drive.deactivateArm();
                    })
                    .lineToConstantHeading(new Vector2d(30, 10), velConPixel, accConPixel)
                    .build();
            trajSeqRight = drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(() ->
                    {
                        drive.initArm();
                        drive.deactivateIntakeServo();
                        drive.activateLift(liftMotorTicks);
                    })
                    .splineToLinearHeading(new Pose2d(10, 30, Math.toRadians(180)), Math.toRadians(180), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(0, () ->
                    {
                        drive.activateArm();
                    })
                    .lineToLinearHeading(new Pose2d(54.75, 28, Math.toRadians(180)), velConBackdrop, accConBackdrop)
                    .waitSeconds(1)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .lineTo(new Vector2d(51.75, 28), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
                    .UNSTABLE_addDisplacementMarkerOffset(0, () ->
                    {
                        drive.restArmAuto();
                        drive.deactivateLift();
                    })
                    .UNSTABLE_addDisplacementMarkerOffset(15, () ->
                    {
                        drive.deactivateArm();
                    })
                    .lineToConstantHeading(new Vector2d(30, 10), velConPixel, accConPixel)
                    .build();
            trajSeqLeft = drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(() ->
                    {
                        drive.initArm();
                        drive.deactivateIntakeServo();
                        drive.activateLift(liftMotorTicks);
                    })
                    .lineTo(new Vector2d(23.5, 42), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(0, () ->
                    {
                        drive.activateArm();
                    })
                    .lineToSplineHeading(new Pose2d(54.75, 40.5, Math.toRadians(180)), velConBackdrop, accConBackdrop)
                    .waitSeconds(1)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .lineTo(new Vector2d(51.75, 40.5), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
                    .UNSTABLE_addDisplacementMarkerOffset(0, () ->
                    {
                        drive.restArmAuto();
                        drive.deactivateLift();
                    })
                    .UNSTABLE_addDisplacementMarkerOffset(15, () ->
                    {
                        drive.deactivateArm();
                    })
                    .lineToConstantHeading(new Vector2d(30, 10), velConPixel, accConPixel)
                    .build();
            trajSeqStackDrive = drive.trajectorySequenceBuilder(trajSeqCenter.end())
                    .addDisplacementMarker(60, () ->
                    {
                        drive.activateIntake();
                    })
                    .lineToConstantHeading(new Vector2d(-55.25, 9), velConPixel, accConPixel)
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->
                    {
                        drive.activateIntakeServo();
                    })
                    .waitSeconds(4)
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->
                    {
                        drive.activateLeftClaw();
                        drive.activateRightClaw();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () ->
                    {
                        drive.deactivateIntakeServo();
                        drive.reverseIntake();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.6, () ->
                    {
                        drive.deactivateIntake();
                    })
                    .waitSeconds(1)
                    .UNSTABLE_addDisplacementMarkerOffset(70, () ->
                    {
                        drive.activateLift(liftMotorTicks);
                        drive.initArm();
                    })
                    .lineToConstantHeading(new Vector2d(52, 13), velConPixel, accConPixel)
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->
                    {
                        drive.activateArm();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.5, () ->
                    {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .waitSeconds(1)
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->
                    {
                        drive.deactivateArm();
                        drive.deactivateLift();
                    })
                    .waitSeconds(1)
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
        drive.followTrajectorySequence(trajSeqStackDrive);
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
