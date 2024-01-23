package org.firstinspires.ftc.teamcode.ftc8468.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "AutoRedHumanPlayer")
public class AutoRedHumanPlayer extends LinearOpMode {
    RRAutoDrive drive;
    String curAlliance = "red";

    private TeamElementSubsystem teamElementDetection;

    private int liftMotorTicks = 425;

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
                        drive.restArmAuto();
                        drive.deactivateIntakeServo();
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
                    .lineToConstantHeading(new Vector2d(52.5, -40), velConPixel, accConPixel)
//                    .splineToConstantHeading(new Vector2d(51.5, -35.5), Math.toRadians(270), velConPixel, accConPixel) //removed spline traj
                    .waitSeconds(1)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1, () ->
                    {
                        drive.restArm();
                    })
                    .lineToConstantHeading(new Vector2d(49.5, -42), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateLift();
                        drive.activateIntakeServo();
                    })
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
                        drive.restArmAuto();
                        drive.deactivateIntakeServo();
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
                    .lineToConstantHeading(new Vector2d(52.5, -32), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1, () ->
                    {
                        drive.restArm();
                    })
                    .lineToConstantHeading(new Vector2d(49.5, -32), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateLift();
                        drive.activateIntakeServo();
                    })
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
                        drive.restArmAuto();
                        drive.deactivateIntakeServo();
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
                    .lineToConstantHeading(new Vector2d(52.5, -45), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1, () ->
                    {
                        drive.restArm();
                    })
                    .lineToConstantHeading(new Vector2d(49.5, -45), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateLift();
                        drive.activateIntakeServo();
                    })
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
        telemetry.addData("Status:", "initialize() - Robot and Camera are initialized!");
        telemetry.update();
    }
}
