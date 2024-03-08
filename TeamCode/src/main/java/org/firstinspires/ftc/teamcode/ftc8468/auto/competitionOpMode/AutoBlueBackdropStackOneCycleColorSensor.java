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
@Autonomous (name = "AutoBlueBackdropStackTwoCycleColorSensor")
public class AutoBlueBackdropStackOneCycleColorSensor extends LinearOpMode {
    RRAutoDrive drive;
    String curAlliance = "blue";

    private TeamElementSubsystem teamElementDetection;

    private int liftMotorTicks = 390;

    enum Wiggle
    {
        LEFT,
        RIGHT,
        NONE
    }

    Wiggle wiggle = Wiggle.LEFT;

    SplitAveragePipeline.ZONE zone = SplitAveragePipeline.ZONE.RIGHT;
    public TrajectorySequence trajSeqCenter, trajSeqLeft, trajSeqRight, trajSeqStackDrive, trajWiggleLeft, trajWiggleRight, trajWiggleLeft2, trajWiggleRight2;
    TrajectoryVelocityConstraint velConPixel = SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    TrajectoryAccelerationConstraint accConPixel = SampleMecanumDrive.getAccelerationConstraint(30);

    TrajectoryVelocityConstraint velConBackdrop = SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    TrajectoryAccelerationConstraint accConBackdrop = SampleMecanumDrive.getAccelerationConstraint(20);
    TrajectoryVelocityConstraint velConSpline = SampleMecanumDrive.getVelocityConstraint(42, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    TrajectoryAccelerationConstraint accConSpline = SampleMecanumDrive.getAccelerationConstraint(25);

    enum State
    {
        FIRST_PATHS,
        FIRST_PATHS_COMPLETED,
        COLLECTION_CYCLE_1_PIXEL_1,
        COLLECTION_CYCLE_1_WIGGLE,
        COLLECTION_CYCLE_1_PIXEL_2,
        DEPOSIT_CYCLE_1_INIT,
        DEPOSIT_CYCLE_1
    }

    State robotState = State.FIRST_PATHS;
    double startTime = 0;

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
                    .waitSeconds(.5)
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
                    .UNSTABLE_addDisplacementMarkerOffset(0, () ->
                    {
                        drive.restArmAuto();
                        drive.deactivateLift();
                    })
                    .UNSTABLE_addDisplacementMarkerOffset(15, () ->
                    {
                        drive.deactivateArm();
                    })
                    .splineToConstantHeading(new Vector2d(40, 11), Math.toRadians(180), velConSpline, accConSpline)
                    .UNSTABLE_addDisplacementMarkerOffset(60, () ->
                    {
                        drive.activateIntake();
                        drive.activateIntakeServo();
                    })
                    .lineToConstantHeading(new Vector2d(-56.7, 11), velConPixel, accConSpline)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        robotState = State.FIRST_PATHS_COMPLETED;
                    })
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
                    .lineToLinearHeading(new Pose2d(54.75, 26, Math.toRadians(180)), velConBackdrop, accConBackdrop)
                    .waitSeconds(.5)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .lineTo(new Vector2d(51.75, 26), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
                    .UNSTABLE_addDisplacementMarkerOffset(0, () ->
                    {
                        drive.restArmAuto();
                        drive.deactivateLift();
                    })
                    .UNSTABLE_addDisplacementMarkerOffset(15, () ->
                    {
                        drive.deactivateArm();
                    })
                    .UNSTABLE_addDisplacementMarkerOffset(0, () ->
                    {
                        drive.restArmAuto();
                        drive.deactivateLift();
                    })
                    .UNSTABLE_addDisplacementMarkerOffset(15, () ->
                    {
                        drive.deactivateArm();
                    })
                    .splineToConstantHeading(new Vector2d(40, 11), Math.toRadians(180), velConSpline, accConSpline)
                    .UNSTABLE_addDisplacementMarkerOffset(60, () ->
                    {
                        drive.activateIntake();
                        drive.activateIntakeServo();
                    })
                    .lineToConstantHeading(new Vector2d(-57, 11), velConPixel, accConSpline)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        robotState = State.FIRST_PATHS_COMPLETED;
                    })
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
                    .waitSeconds(.5)
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
                    .UNSTABLE_addDisplacementMarkerOffset(0, () ->
                    {
                        drive.restArmAuto();
                        drive.deactivateLift();
                    })
                    .UNSTABLE_addDisplacementMarkerOffset(15, () ->
                    {
                        drive.deactivateArm();
                    })
                    .splineToConstantHeading(new Vector2d(40, 11), Math.toRadians(180), velConSpline, accConSpline)
                    .UNSTABLE_addDisplacementMarkerOffset(60, () ->
                    {
                        drive.activateIntake();
                        drive.activateIntakeServo();
                    })
                    .lineToConstantHeading(new Vector2d(-57, 11), velConPixel, accConSpline)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        robotState = State.FIRST_PATHS_COMPLETED;
                    })
                    .build();
            trajWiggleLeft = drive.trajectorySequenceBuilder(trajSeqCenter.end())
                    .turn(Math.toRadians(30))
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->{
                        startTime = drive.elapsedSeconds();
                        robotState = State.COLLECTION_CYCLE_1_PIXEL_2;
                        wiggle = Wiggle.LEFT;
                    })
                    .build();
            trajWiggleRight = drive.trajectorySequenceBuilder(trajSeqCenter.end())
                    .turn(Math.toRadians(-30))
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->{
                        startTime = drive.elapsedSeconds();
                        robotState = State.COLLECTION_CYCLE_1_PIXEL_2;
                        wiggle = Wiggle.RIGHT;
                    })
                    .build();
        }
        else {
            // do nothing!
        }

        waitForStart();
        drive.resetRuntime();

        if (isStopRequested()) return;
        if (zone == SplitAveragePipeline.ZONE.LEFT)
            drive.followTrajectorySequenceAsync(trajSeqLeft);
        else if (zone == SplitAveragePipeline.ZONE.CENTER)
            drive.followTrajectorySequenceAsync(trajSeqCenter);
        else
            drive.followTrajectorySequenceAsync(trajSeqRight);

        while (opModeIsActive() && !isStopRequested())
        {
            if (robotState == State.FIRST_PATHS_COMPLETED)
            {
                startTime = drive.elapsedSeconds();
                robotState = State.COLLECTION_CYCLE_1_PIXEL_1;
            }
            if (robotState == State.COLLECTION_CYCLE_1_PIXEL_1)
            {
                if (drive.getPixelCount() == RRAutoDrive.PixelCount.ONE)
                {
                    if (drive.leftPixelContained()) {
                        robotState = State.COLLECTION_CYCLE_1_WIGGLE;
                        drive.followTrajectorySequenceAsync(trajWiggleLeft);
                    }
                    if (drive.rightPixelContained()) {
                        robotState = State.COLLECTION_CYCLE_1_WIGGLE;
                        drive.followTrajectorySequenceAsync(trajWiggleRight);
                    }
                }

                if (drive.elapsedSeconds() - startTime > 1.5)
                {
                    robotState = State.COLLECTION_CYCLE_1_WIGGLE;
                    drive.followTrajectorySequenceAsync(trajWiggleLeft);
                }
            }
            if (robotState == State.COLLECTION_CYCLE_1_PIXEL_2)
            {
                if (drive.getPixelCount() == RRAutoDrive.PixelCount.TWO) {
                    robotState = State.DEPOSIT_CYCLE_1_INIT;
                }

                if (drive.elapsedSeconds() - startTime > 0.5)
                    robotState = State.DEPOSIT_CYCLE_1_INIT;
            }
            if (robotState == State.DEPOSIT_CYCLE_1_INIT)
            {
                Pose2d drivePose = (wiggle == Wiggle.LEFT ? trajWiggleLeft.end(): trajWiggleRight.end());
                trajSeqStackDrive = drive.trajectorySequenceBuilder(drivePose)
                        .UNSTABLE_addDisplacementMarkerOffset(0, () ->
                        {
                            drive.reverseIntake();
                        })
                        .UNSTABLE_addDisplacementMarkerOffset(10, () ->
                        {
                            drive.deactivateIntake();
                        })
                        .UNSTABLE_addDisplacementMarkerOffset(50, () ->
                        {
                            drive.activateLift(liftMotorTicks);
                            drive.initArm();
                        })
                        .UNSTABLE_addDisplacementMarkerOffset(70, () ->
                        {
                            drive.activateArm();
                        })
                        .lineToLinearHeading(new Pose2d(56, 11, Math.toRadians(180)), velConPixel, accConPixel)
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
                        .build();
                drive.followTrajectorySequenceAsync(trajSeqStackDrive);
                robotState = State.DEPOSIT_CYCLE_1;
            }

            drive.update();
        }
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
