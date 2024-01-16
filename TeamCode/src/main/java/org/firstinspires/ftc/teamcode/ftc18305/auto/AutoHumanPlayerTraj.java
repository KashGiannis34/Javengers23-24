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

@Autonomous (name = "TrajTest2HumanPlayer")
public class AutoHumanPlayerTraj extends LinearOpMode {
    RRAutoDrive drive;
    String curAlliance = "red";

    private TeamElementSubsystem teamElementDetection;

    SplitAveragePipeline.ZONE zone = SplitAveragePipeline.ZONE.RIGHT;
    TrajectorySequence trajSeqCenter, trajSeqLeft, trajSeqRight;
    TrajectoryVelocityConstraint velCon = SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    TrajectoryAccelerationConstraint accCon = SampleMecanumDrive.getAccelerationConstraint(40);

    public void runOpMode()
    {
        initialize();
        Pose2d startPose = new Pose2d(-42, 64, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        trajSeqCenter = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-36, 30), velCon, accCon) // drive to team prop
                .lineToSplineHeading(new Pose2d(-36, 42, Math.toRadians(180)), velCon, accCon) // rotate 180 and drive to truss entrance
                .splineToConstantHeading(new Vector2d(30, 54), Math.toRadians(0), velCon, accCon) // drive through truss in spline
                .splineToConstantHeading(new Vector2d(40, 36), Math.toRadians(0), velCon, accCon) // spline part 2; end up at backdrop
                .lineTo(new Vector2d(40, 20), velCon, accCon) // park path part 1
                .splineToConstantHeading(new Vector2d(60, 8), Math.toRadians(0), velCon, accCon) // park path sequel
                .build();
        trajSeqRight = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-42, 34, Math.toRadians(220)), velCon, accCon) // drive to team prop with rotation
                .lineTo(new Vector2d(-40.5,41.5), velCon, accCon) // back off no rotation
                .lineToSplineHeading(new Pose2d(-40, 44, Math.toRadians(180)), velCon, accCon) // back off with rotation
                .splineToConstantHeading(new Vector2d(0, 58), Math.toRadians(0), velCon, accCon) // spline to truss entrance
                .lineTo(new Vector2d(25, 58), velCon, accCon)  // drive through truss
                .splineToConstantHeading(new Vector2d(40, 30), Math.toRadians(0), velCon, accCon) // backdrop
                .lineTo(new Vector2d(40, 20), velCon, accCon) // park salaar chapter 1
                .splineToConstantHeading(new Vector2d(60, 8), Math.toRadians(0), velCon, accCon) // park salaar part 2
                .build();
        trajSeqLeft = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-26, 34, Math.toRadians(300)), velCon, accCon) // drive to prop with rotation
                .lineTo(new Vector2d(-33, 41.5), velCon, accCon) // back off without rotation
                .lineToSplineHeading(new Pose2d(-40, 49, Math.toRadians(180)), velCon, accCon) // back off with rotation
                .splineToConstantHeading(new Vector2d(0, 58), Math.toRadians(0), velCon, accCon) // spline to truss entrance
                .lineTo(new Vector2d(25, 58), velCon, accCon) // drive through truss
                .splineToConstantHeading(new Vector2d(40, 42), Math.toRadians(0), velCon, accCon) // spline to backdrop
                .lineTo(new Vector2d(40, 20), velCon, accCon) // park numero uno
                .splineToConstantHeading(new Vector2d(60, 8), Math.toRadians(0), velCon, accCon) // park numero dosa
                .build();

        teamElementDetection = new TeamElementSubsystem(hardwareMap, SplitAveragePipeline.ZONE_VIEW.RIGHT);

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
        telemetry.addData("Status:", "initialize() - Robot and Camera are initialized!");
        telemetry.update();
    }
}
