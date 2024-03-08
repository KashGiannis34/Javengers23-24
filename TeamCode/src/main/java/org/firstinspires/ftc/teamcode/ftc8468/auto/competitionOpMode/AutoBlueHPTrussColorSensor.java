package org.firstinspires.ftc.teamcode.ftc8468.auto.competitionOpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ftc8468.auto.RRAutoDrive;
import org.firstinspires.ftc.teamcode.ftc8468.auto.pipelines.SplitAveragePipeline;
import org.firstinspires.ftc.teamcode.ftc8468.auto.pipelines.TeamElementSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous (name = "AutoBlueHPTrussAprilTag")
public class AutoBlueHPTrussColorSensor extends LinearOpMode {
    RRAutoDrive drive;
    String curAlliance = "blue";

    private TeamElementSubsystem teamElementDetection;

    private int liftMotorTicks = 440;

    SplitAveragePipeline.ZONE zone = SplitAveragePipeline.ZONE.RIGHT;
    TrajectorySequence trajSeqCenter, trajSeqLeft, trajSeqRight;
    TrajectoryVelocityConstraint velCon = SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    TrajectoryAccelerationConstraint accCon = SampleMecanumDrive.getAccelerationConstraint(40);
    TrajectoryVelocityConstraint velConPixel = SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    TrajectoryAccelerationConstraint accConPixel = SampleMecanumDrive.getAccelerationConstraint(15);

    enum State
    {
        PATH_FOLLOWING,
        DETECT_PIXEL
    }

    State robotState;
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
                        drive.initArm();
                        drive.deactivateIntakeServo();
                        drive.activateLift(liftMotorTicks);
                    })
                    .lineToConstantHeading(new Vector2d(-39, 30), velConPixel, accConPixel)
                    .lineToLinearHeading(new Pose2d(-42, 56, Math.toRadians(180)), velConPixel, accConPixel)
                    .waitSeconds(4)
                    .lineTo(new Vector2d(24, 56), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(24, () ->
                    {
                        drive.activateArm();
                    })
                    .lineTo(new Vector2d(50, 32), velConPixel, accConPixel)
                    .build();
            trajSeqRight = drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(() ->
                    {
                        drive.initArm();
                        drive.deactivateIntakeServo();
                        drive.activateLift(liftMotorTicks);
                    })
                    .lineToLinearHeading(new Pose2d(-42, 34, Math.toRadians(240)), velConPixel, accConPixel)
                    .lineToLinearHeading(new Pose2d(-42, 40, Math.toRadians(270)), velConPixel, accConPixel)
                    .lineToLinearHeading(new Pose2d(-42, 58, Math.toRadians(180)), velConPixel, accConPixel)
                    .waitSeconds(3)
                    .lineTo(new Vector2d(24, 58), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(24, () ->
                    {
                        drive.activateArm();
                    })
                    .lineTo(new Vector2d(50, 28), velConPixel, accConPixel)
                    .build();
            trajSeqLeft = drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(() ->
                    {
                        drive.initArm();
                        drive.deactivateIntakeServo();
                        drive.activateLift(liftMotorTicks);
                    })
                    .lineToLinearHeading(new Pose2d(-32, 32, Math.toRadians(340)), velConPixel, accConPixel)
                    .lineToLinearHeading(new Pose2d(-39, 42, Math.toRadians(270)), velConPixel, accConPixel)
                    .lineToLinearHeading(new Pose2d(-42, 58, Math.toRadians(180)), velConPixel, accConPixel)
                    .waitSeconds(3)
                    .lineTo(new Vector2d(24, 58), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(24, () ->
                    {
                        drive.activateArm();
                    })
                    .lineTo(new Vector2d(50, 37), velConPixel, accConPixel)
                    .build();
        }
        else {
            // do nothing!
        }

        waitForStart();
        if (isStopRequested()) return;
        if (zone == SplitAveragePipeline.ZONE.LEFT)
            drive.followTrajectorySequenceAsync(trajSeqLeft);
        else if (zone == SplitAveragePipeline.ZONE.CENTER)
            drive.followTrajectorySequenceAsync(trajSeqCenter);
        else
            drive.followTrajectorySequenceAsync(trajSeqRight);

        robotState = State.PATH_FOLLOWING;

        while (opModeIsActive() && !isStopRequested())
        {
            if (drive.getPoseEstimate().getX() > 49)
            {
                drive.breakFollowing();
                drive.setDrivePower(new Pose2d());
            }

            drive.update();
        }

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
