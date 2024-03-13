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

@Autonomous (name = "AutoRedHPTruss")
public class AutoRedHPTruss extends LinearOpMode {
    RRAutoDrive drive;
    String curAlliance = "red";

    private TeamElementSubsystem teamElementDetection;

    private int liftMotorTicks = 495;

    SplitAveragePipeline.ZONE zone = SplitAveragePipeline.ZONE.RIGHT;
    TrajectorySequence trajSeqCenter, trajSeqLeft, trajSeqRight;
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

        if (curAlliance == "red") {
            Pose2d startPose = new Pose2d(-42, -64, Math.toRadians(90));

            drive.setPoseEstimate(startPose);

            trajSeqCenter = drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(() ->
                    {
                        drive.restArm();
                        drive.deactivateIntakeServo();
                        drive.activateLift(liftMotorTicks);
                    })
                    .lineToConstantHeading(new Vector2d(-39, -31.5), velConPixel, accConPixel) // no-spline traj
                    .lineToConstantHeading(new Vector2d(-39, -42), velConPixel, accConPixel)
                    .lineToLinearHeading(new Pose2d(-42, -58, Math.toRadians(180)), velConPixel, accConPixel)
                    .waitSeconds(4)
                    .lineTo(new Vector2d(24, -58), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(24, () ->
                    {;
                        drive.activateArm();
                    })
                    .lineTo(new Vector2d(49, -38), velConPixel, accConPixel)
                    .lineTo(new Vector2d(52, -38), velConPixel, accConPixel)
                    .waitSeconds(1)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .lineTo(new Vector2d(49, -38), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->
                    {
                        drive.restArmAuto();
                        drive.activateIntakeServo();
                        drive.deactivateLift();
                    })
                    .lineTo(new Vector2d(49, -57), velConPixel, accConPixel)
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
                        drive.deactivateIntakeServo();
                        drive.activateLift(liftMotorTicks);
                    })
                    .lineToLinearHeading(new Pose2d(-42, -37.5, Math.toRadians(120)), velConPixel, accConPixel)
                    .lineToLinearHeading(new Pose2d(-42, -56, Math.toRadians(180)), velConPixel, accConPixel)
                    .waitSeconds(3)
                    .lineTo(new Vector2d(24, -56), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(24, () ->
                    {
                        drive.activateArm();
                    })
                    .lineTo(new Vector2d(49, -33), velConPixel, accConPixel)
                    .lineTo(new Vector2d(54, -33), velConPixel, accConPixel)
                    .waitSeconds(1)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .lineTo(new Vector2d(49, -31.5), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->
                    {
                        drive.restArmAuto();
                        drive.activateIntakeServo();
                        drive.deactivateLift();
                    })
                    .lineTo(new Vector2d(49, -60), velConPixel, accConPixel)
                    .build();
            trajSeqRight = drive.trajectorySequenceBuilder(startPose)
                   .addTemporalMarker(() ->
                    {
                        drive.restArm();
                        drive.deactivateIntakeServo();
                        drive.activateLift(liftMotorTicks);
                    })
                    .lineToLinearHeading(new Pose2d(-32, -32, Math.toRadians(20)), velConPixel, accConPixel)
                    .lineToLinearHeading(new Pose2d(-42, -58, Math.toRadians(180)), velConPixel, accConPixel)
                    .waitSeconds(3)
                    .lineTo(new Vector2d(24, -58), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(24, () ->
                    {
                        drive.activateArm();
                    })
                    .lineTo(new Vector2d(49, -40), velConPixel, accConPixel)
                    .lineTo(new Vector2d(52, -40), velConPixel, accConPixel)
                    .waitSeconds(1)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .lineTo(new Vector2d(49, -40), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->
                    {
                        drive.restArmAuto();
                        drive.activateIntakeServo();
                        drive.deactivateLift();
                    })
                    .lineTo(new Vector2d(49, -57), velConPixel, accConPixel)
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
