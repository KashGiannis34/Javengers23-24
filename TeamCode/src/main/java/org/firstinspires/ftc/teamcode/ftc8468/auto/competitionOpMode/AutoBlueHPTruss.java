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

@Autonomous (name = "AutoBlueHPTruss")
public class AutoBlueHPTruss extends LinearOpMode {
    RRAutoDrive drive;
    String curAlliance = "blue";

    private TeamElementSubsystem teamElementDetection;

    private int liftMotorTicks = 550;

    SplitAveragePipeline.ZONE zone = SplitAveragePipeline.ZONE.RIGHT;
    TrajectorySequence trajSeqCenter, trajSeqLeft, trajSeqRight;
    TrajectoryVelocityConstraint velCon = SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    TrajectoryAccelerationConstraint accCon = SampleMecanumDrive.getAccelerationConstraint(40);
    TrajectoryVelocityConstraint velConPixel = SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    TrajectoryAccelerationConstraint accConPixel = SampleMecanumDrive.getAccelerationConstraint(15);

    public void runOpMode()
    {
        initialize();

        teamElementDetection = new TeamElementSubsystem(hardwareMap, SplitAveragePipeline.ZONE_VIEW.RIGHT);
        while (!isStarted() && !isStopRequested())
        {
            zone = teamElementDetection.getElementZoneValue(telemetry);
//            if (gamepad1.x){
//                curAlliance = "blue";
//                teamElementDetection.setZoneView(SplitAveragePipeline.ZONE_VIEW.RIGHT);
//            }else if (gamepad1.b){
//                curAlliance = "red";
//                teamElementDetection.setZoneView(SplitAveragePipeline.ZONE_VIEW.LEFT);
//            }
            teamElementDetection.setAlliance(curAlliance);
//            telemetry.addData("Select Alliance (Gamepad1 X = Blue, Gamepad1 B = Red)", "");
            telemetry.addData("Current Alliance Selected : ", curAlliance.toUpperCase());
            telemetry.update();
        }

        if (curAlliance == "blue") {
            Pose2d startPose = new Pose2d(-42, 64, Math.toRadians(270));

            drive.setPoseEstimate(startPose);

            trajSeqCenter = drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(() ->
                    {
                        drive.restArmAuto();
                        drive.deactivateIntakeServo();
                    })
                    .lineToConstantHeading(new Vector2d(-39, 30), velConPixel, accConPixel)
                    .lineToLinearHeading(new Pose2d(-42, 56, Math.toRadians(180)), velConPixel, accConPixel)
                    .waitSeconds(4)
                    .lineTo(new Vector2d(24, 56), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(24, () ->
                    {
                        drive.activateLift(liftMotorTicks);
                        drive.activateArm();
                    })
                    .lineTo(new Vector2d(49, 32), velConPixel, accConPixel)
                    .lineTo(new Vector2d(52, 32), velConPixel, accConPixel)
                    .waitSeconds(1)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .lineTo(new Vector2d(48, 32), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->
                    {
                        drive.restArmAuto();
                        drive.activateIntakeServo();
                        drive.deactivateLift();
                    })
                    .lineTo(new Vector2d(56, 50), velConPixel, accConPixel)
                    .build();
            trajSeqRight = drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(() ->
                    {
                        drive.restArmAuto();;
                        drive.deactivateIntakeServo();
                    })
                    .lineToLinearHeading(new Pose2d(-42, 34, Math.toRadians(240)), velConPixel, accConPixel)
                    .lineToLinearHeading(new Pose2d(-42, 40, Math.toRadians(270)), velConPixel, accConPixel)
                    .lineToLinearHeading(new Pose2d(-42, 58, Math.toRadians(180)), velConPixel, accConPixel)
                    .waitSeconds(3)
                    .lineTo(new Vector2d(24, 58), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(24, () ->
                    {
                        drive.activateLift(liftMotorTicks);
                        drive.activateArm();
                    })
                    .lineTo(new Vector2d(49, 28), velConPixel, accConPixel)
                    .lineTo(new Vector2d(52, 28), velConPixel, accConPixel)
                    .waitSeconds(1)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .lineTo(new Vector2d(48, 28), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->
                    {
                        drive.restArmAuto();
                        drive.activateIntakeServo();
                        drive.deactivateLift();
                    })
                    .lineTo(new Vector2d(56, 50), velConPixel, accConPixel)
                    .build();
            trajSeqLeft = drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(() ->
                    {
                        drive.restArmAuto();;
                        drive.deactivateIntakeServo();
                    })
                    .lineToLinearHeading(new Pose2d(-32, 32, Math.toRadians(340)), velConPixel, accConPixel)
                    .lineToLinearHeading(new Pose2d(-39, 42, Math.toRadians(270)), velConPixel, accConPixel)
                    .lineToLinearHeading(new Pose2d(-42, 58, Math.toRadians(180)), velConPixel, accConPixel)
                    .waitSeconds(3)
                    .lineTo(new Vector2d(24, 58), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(24, () ->
                    {
                        drive.activateLift(liftMotorTicks);
                        drive.activateArm();
                    })
                    .lineTo(new Vector2d(49, 39), velConPixel, accConPixel)
                    .lineTo(new Vector2d(52, 39), velConPixel, accConPixel)
                    .waitSeconds(1)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .lineTo(new Vector2d(48, 37), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->
                    {
                        drive.restArmAuto();
                        drive.activateIntakeServo();
                        drive.deactivateLift();
                    })
                    .lineTo(new Vector2d(56, 50), velConPixel, accConPixel)

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
        drive.deactivateIntakeServo();
        drive.initArm();
        drive.activateLeftClaw();
        drive.activateRightClaw();
        telemetry.addData("Status:", "initialize() - Robot and Camera are initialized!");
        telemetry.update();
    }
}
