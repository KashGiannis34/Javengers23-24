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
@Autonomous (name = "AutoRedBackdropStackTwoCycle")
public class AutoRedBackdropStackTwoCycle extends LinearOpMode {
    RRAutoDrive drive;
    String curAlliance = "red";

    private TeamElementSubsystem teamElementDetection;

    private int liftMotorTicks = 400;

    SplitAveragePipeline.ZONE zone = SplitAveragePipeline.ZONE.RIGHT;
    public TrajectorySequence trajSeqCenter, trajSeqLeft, trajSeqRight, trajSeqStackDrive;
    TrajectoryVelocityConstraint velConPixel = SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    TrajectoryAccelerationConstraint accConPixel = SampleMecanumDrive.getAccelerationConstraint(34);

    TrajectoryVelocityConstraint velConBackdrop = SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    TrajectoryAccelerationConstraint accConBackdrop = SampleMecanumDrive.getAccelerationConstraint(20);

    TrajectoryVelocityConstraint velConSpline = SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    TrajectoryAccelerationConstraint accConSpline = SampleMecanumDrive.getAccelerationConstraint(20);

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
                        drive.activateLift(liftMotorTicks);
                    })
                    .lineTo(new Vector2d(18, -32), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(0, () ->
                    {
                        drive.activateArm();
                    })
                    .lineToLinearHeading(new Pose2d(55.25, -36.25, Math.toRadians(180)), velConBackdrop, accConBackdrop)
                    .waitSeconds(.5)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .lineTo(new Vector2d(51, -36.25), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
                    .UNSTABLE_addDisplacementMarkerOffset(0, () ->
                    {
                        drive.restArmAuto();
                        drive.deactivateLift();
                    })
                    .UNSTABLE_addDisplacementMarkerOffset(15, () ->
                    {
                        drive.deactivateArm();
                    })
                    .splineToConstantHeading(new Vector2d(40, -12.45), Math.toRadians(180), velConSpline, accConSpline)
                    .UNSTABLE_addDisplacementMarkerOffset(60, () ->
                    {
                        drive.activateIntake();
                        drive.activateIntakeServo();
                    })
                    .lineToConstantHeading(new Vector2d(-56.7, -12.45), velConPixel, accConPixel)
                    .waitSeconds(1)
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
                    .waitSeconds(.3)
                    .UNSTABLE_addDisplacementMarkerOffset(50, () ->
                    {
                        drive.activateLift(liftMotorTicks);
                        drive.initArm();
                    })
                    .UNSTABLE_addDisplacementMarkerOffset(70, () ->
                    {
                        drive.activateArm();
                    })
                    .lineToConstantHeading(new Vector2d(56, -13), velConPixel, accConPixel)
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->
                    {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .waitSeconds(.2)
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->
                    {
                        drive.restArmAuto();
                        drive.deactivateLift();
                    })
                    .UNSTABLE_addDisplacementMarkerOffset(15, () ->
                    {
                        drive.deactivateArm();
                    })
                    .UNSTABLE_addDisplacementMarkerOffset(60, () ->
                    {
                        drive.activateIntakeServoThree();
                        drive.activateIntake();
                    })
                    .lineToConstantHeading(new Vector2d(-57.5, -14), velConPixel, accConPixel)
                    .UNSTABLE_addTemporalMarkerOffset(0.23, () ->
                    {
                        drive.activateIntakeServoTwo();
                        drive.activateIntake();
                    })
                    .waitSeconds(1)
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
                    .waitSeconds(.3)
                    .UNSTABLE_addDisplacementMarkerOffset(50, () ->
                    {
                        drive.activateLift(liftMotorTicks);
                        drive.initArm();
                    })
                    .UNSTABLE_addDisplacementMarkerOffset(70, () ->
                    {
                        drive.activateArm();
                    })
                    .lineToConstantHeading(new Vector2d(56.15, -13), velConPixel, accConPixel)
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->
                    {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .waitSeconds(.2)
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->
                    {
                        drive.restArmAuto();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(.3, () ->
                    {
                        drive.deactivateLift();
                    })
                    .waitSeconds(.6)
                    .build();
            trajSeqLeft = drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(() ->
                    {
                        drive.initArm();
                        drive.deactivateIntakeServo();
                        drive.activateLift(liftMotorTicks);
                    })
                    .splineToLinearHeading(new Pose2d(10, -30, Math.toRadians(180)), Math.toRadians(180), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(0, () ->
                    {
                        drive.activateArm();
                    })
                    .lineToLinearHeading(new Pose2d(55.25, -31, Math.toRadians(180)), velConBackdrop, accConBackdrop)
                    .waitSeconds(.5)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .lineTo(new Vector2d(51, -31), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
                    .UNSTABLE_addDisplacementMarkerOffset(0, () ->
                    {
                        drive.restArmAuto();
                        drive.deactivateLift();
                    })
                    .UNSTABLE_addDisplacementMarkerOffset(15, () ->
                    {
                        drive.deactivateArm();
                    })
                    .splineToConstantHeading(new Vector2d(40, -12.45), Math.toRadians(180), velConSpline, accConSpline)
                    .UNSTABLE_addDisplacementMarkerOffset(60, () ->
                    {
                        drive.activateIntake();
                        drive.activateIntakeServo();
                    })
                    .lineToConstantHeading(new Vector2d(-57.5, -12.75), velConPixel, accConPixel)
                    .waitSeconds(1)
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
                    .waitSeconds(.3)
                    .UNSTABLE_addDisplacementMarkerOffset(50, () ->
                    {
                        drive.activateLift(liftMotorTicks);
                        drive.initArm();
                    })
                    .UNSTABLE_addDisplacementMarkerOffset(70, () ->
                    {
                        drive.activateArm();
                    })
                    .lineToConstantHeading(new Vector2d(56, -13), velConPixel, accConPixel)
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->
                    {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .waitSeconds(.2)
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->
                    {
                        drive.restArmAuto();
                        drive.deactivateLift();
                    })
                    .UNSTABLE_addDisplacementMarkerOffset(15, () ->
                    {
                        drive.deactivateArm();
                    })
                    .UNSTABLE_addDisplacementMarkerOffset(60, () ->
                    {
                        drive.activateIntakeServoThree();
                        drive.activateIntake();
                    })
                    .lineToConstantHeading(new Vector2d(-57.75, -14.3), velConPixel, accConPixel)
                    .UNSTABLE_addTemporalMarkerOffset(0.23, () ->
                    {
                        drive.activateIntakeServoTwo();
                        drive.activateIntake();
                    })
                    .waitSeconds(1)
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
                    .waitSeconds(.3)
                    .UNSTABLE_addDisplacementMarkerOffset(50, () ->
                    {
                        drive.activateLift(liftMotorTicks);
                        drive.initArm();
                    })
                    .UNSTABLE_addDisplacementMarkerOffset(70, () ->
                    {
                        drive.activateArm();
                    })
                    .lineToConstantHeading(new Vector2d(56.15, -13), velConPixel, accConPixel)
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->
                    {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .waitSeconds(.2)
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->
                    {
                        drive.restArmAuto();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(.3, () ->
                    {
                        drive.deactivateLift();
                    })
                    .waitSeconds(.6)
                    .build();
            trajSeqRight = drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(() ->
                    {
                        drive.initArm();
                        drive.deactivateIntakeServo();
                        drive.activateLift(liftMotorTicks);
                    })
                    .lineTo(new Vector2d(25, -38), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(0, () ->
                    {
                        drive.activateArm();
                    })
                    .lineToSplineHeading(new Pose2d(55.25, -40.5, Math.toRadians(180)), velConBackdrop, accConBackdrop)
                    .waitSeconds(1)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .lineTo(new Vector2d(51, -40.5), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
                    .UNSTABLE_addDisplacementMarkerOffset(0, () ->
                    {
                        drive.restArmAuto();
                        drive.deactivateLift();
                    })
                    .UNSTABLE_addDisplacementMarkerOffset(15, () ->
                    {
                        drive.deactivateArm();
                    })
                    .splineToConstantHeading(new Vector2d(40, -12.45), Math.toRadians(180), velConSpline, accConSpline)
                    .UNSTABLE_addDisplacementMarkerOffset(60, () ->
                    {
                        drive.activateIntake();
                        drive.activateIntakeServo();
                    })
                    .lineToConstantHeading(new Vector2d(-56.7, -12.45), velConPixel, accConPixel)
                    .waitSeconds(1)
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
                    .waitSeconds(.3)
                    .UNSTABLE_addDisplacementMarkerOffset(50, () ->
                    {
                        drive.activateLift(liftMotorTicks);
                        drive.initArm();
                    })
                    .UNSTABLE_addDisplacementMarkerOffset(70, () ->
                    {
                        drive.activateArm();
                    })
                    .lineToConstantHeading(new Vector2d(56, -13), velConPixel, accConPixel)
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->
                    {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .waitSeconds(.2)
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->
                    {
                        drive.restArmAuto();
                        drive.deactivateLift();
                    })
                    .UNSTABLE_addDisplacementMarkerOffset(15, () ->
                    {
                        drive.deactivateArm();
                    })
                    .UNSTABLE_addDisplacementMarkerOffset(60, () ->
                    {
                        drive.activateIntakeServoThree();
                        drive.activateIntake();
                    })
                    .lineToConstantHeading(new Vector2d(-57.5, -14), velConPixel, accConPixel)
                    .UNSTABLE_addTemporalMarkerOffset(0.23, () ->
                    {
                        drive.activateIntakeServoTwo();
                        drive.activateIntake();
                    })
                    .waitSeconds(1)
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
                    .waitSeconds(.3)
                    .UNSTABLE_addDisplacementMarkerOffset(50, () ->
                    {
                        drive.activateLift(liftMotorTicks);
                        drive.initArm();
                    })
                    .UNSTABLE_addDisplacementMarkerOffset(70, () ->
                    {
                        drive.activateArm();
                    })
                    .lineToConstantHeading(new Vector2d(56.15, -13), velConPixel, accConPixel)
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->
                    {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .waitSeconds(.2)
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->
                    {
                        drive.restArmAuto();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(.3, () ->
                    {
                        drive.deactivateLift();
                    })
                    .waitSeconds(.6)
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
