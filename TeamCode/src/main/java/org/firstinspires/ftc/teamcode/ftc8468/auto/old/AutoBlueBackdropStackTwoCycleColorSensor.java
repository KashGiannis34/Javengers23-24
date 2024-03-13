package org.firstinspires.ftc.teamcode.ftc8468.auto.old;

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
public class AutoBlueBackdropStackTwoCycleColorSensor extends LinearOpMode {
    RRAutoDrive drive;
    String curAlliance = "blue";

    RRAutoDrive.PixelCount pixelCount;

    private TeamElementSubsystem teamElementDetection;

    private int liftMotorTicks = 390;

    enum Wiggle
    {
        LEFT,
        RIGHT,
        NO_PIXELS,
        NONE
    }

    Wiggle wiggle = Wiggle.LEFT;

    SplitAveragePipeline.ZONE zone = SplitAveragePipeline.ZONE.RIGHT;
    public TrajectorySequence trajSeqCenter, trajSeqLeft, trajSeqRight, trajSeqStackDrive, trajWiggleLeft, trajWiggleRight, trajWiggleNoPixels, trajSecondCycle;
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
        COLLECTION_CYCLE_1_PIXEL_1_IN,
        COLLECTION_CYCLE_1_WIGGLE,
        COLLECTION_CYCLE_1_PIXEL_2,
        DEPOSIT_CYCLE_1_INIT,
        DEPOSIT_CYCLE_1,
        FIRST_PATHS_AGAIN_INIT,
        FIRST_PATHS_AGAIN
    }

    State robotState = State.FIRST_PATHS;
    double startTime = 0;
    double startTimeOnePixel = 0;

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
                    .lineToLinearHeading(new Pose2d(55.75, 32, Math.toRadians(180)), velConBackdrop, accConBackdrop)
                    .waitSeconds(.5)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .lineToConstantHeading(new Vector2d(51.75, 32), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
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
                    .lineToLinearHeading(new Pose2d(55.75, 26, Math.toRadians(180)), velConBackdrop, accConBackdrop)
                    .waitSeconds(.5)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .lineToConstantHeading(new Vector2d(51.75, 26), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
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
                    .lineToSplineHeading(new Pose2d(55.75, 40.5, Math.toRadians(180)), velConBackdrop, accConBackdrop)
                    .waitSeconds(.5)
                    .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .lineToConstantHeading(new Vector2d(51.75, 40.5), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
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
                    .addDisplacementMarker(0, () -> {
                        drive.activateIntakeServoTwo();
                    })
                    .turn(Math.toRadians(30))
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->{
                        startTime = drive.elapsedSeconds();
                        robotState = State.COLLECTION_CYCLE_1_PIXEL_2;
                        wiggle = Wiggle.LEFT;
                    })
                    .build();
            trajWiggleRight = drive.trajectorySequenceBuilder(trajSeqCenter.end())
                    .addDisplacementMarker(0, () -> {
                        drive.activateIntakeServoTwo();
                    })
                    .turn(Math.toRadians(-30))
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->{
                        startTime = drive.elapsedSeconds();
                        robotState = State.COLLECTION_CYCLE_1_PIXEL_2;
                        wiggle = Wiggle.RIGHT;
                    })
                    .build();
            trajWiggleNoPixels = drive.trajectorySequenceBuilder(trajSeqCenter.end())
                    .addDisplacementMarker(0, () -> {
                        drive.activateIntakeServoTwo();
                    })
                    .forward(2)
                    .waitSeconds(.25)
                    .back(2)
                    .turn(Math.toRadians(-30))
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->{
                        startTime = drive.elapsedSeconds();
                        robotState = State.COLLECTION_CYCLE_1_PIXEL_2;
                        wiggle = Wiggle.NO_PIXELS;
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
            pixelCount = drive.getPixelCount();
            if (robotState == State.FIRST_PATHS_AGAIN_INIT)
            {
                if (drive.elapsedSeconds() > 22)
                    break;
                trajSecondCycle = drive.trajectorySequenceBuilder(trajSeqStackDrive.end())
                        .addDisplacementMarker(60, () ->
                        {
                            drive.activateIntake();
                            drive.activateIntakeServoThree();
                        })
                        .lineToConstantHeading(new Vector2d(-57, 10), velConPixel, accConSpline)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            robotState = State.FIRST_PATHS_COMPLETED;
                        })
                        .build();
                drive.followTrajectorySequenceAsync(trajSecondCycle);
                robotState = State.FIRST_PATHS_AGAIN;
            }
            if (robotState == State.FIRST_PATHS_COMPLETED)
            {
                startTime = drive.elapsedSeconds();
                robotState = State.COLLECTION_CYCLE_1_PIXEL_1;
            }
            if (robotState == State.COLLECTION_CYCLE_1_PIXEL_1)
            {
                if (pixelCount == RRAutoDrive.PixelCount.TWO)
                {
                    wiggle = Wiggle.NONE;
                    robotState = State.COLLECTION_CYCLE_1_PIXEL_2;
                }

                if (pixelCount == RRAutoDrive.PixelCount.ONE)
                {
                    startTimeOnePixel = drive.elapsedSeconds();
                    robotState = State.COLLECTION_CYCLE_1_PIXEL_1_IN;
                }

                if (drive.elapsedSeconds() - startTime > 2 && pixelCount == RRAutoDrive.PixelCount.ZERO)
                {
                    robotState = State.COLLECTION_CYCLE_1_WIGGLE;
                    drive.followTrajectorySequenceAsync(trajWiggleNoPixels);
                }
            }
            if (robotState == State.COLLECTION_CYCLE_1_PIXEL_1_IN)
            {
                if (pixelCount == RRAutoDrive.PixelCount.TWO)
                {
                    wiggle = Wiggle.NONE;
                    robotState = State.COLLECTION_CYCLE_1_PIXEL_2;
                }

                if (drive.elapsedSeconds() - startTimeOnePixel > .5 && pixelCount == RRAutoDrive.PixelCount.ONE)
                {
                    if (drive.leftPixelContained() && pixelCount == RRAutoDrive.PixelCount.ONE) {
                        robotState = State.COLLECTION_CYCLE_1_WIGGLE;
                        drive.followTrajectorySequenceAsync(trajWiggleLeft);
                    }
                    if (drive.rightPixelContained() && pixelCount == RRAutoDrive.PixelCount.ONE) {
                        robotState = State.COLLECTION_CYCLE_1_WIGGLE;
                        drive.followTrajectorySequenceAsync(trajWiggleRight);
                    }
                }

                if (drive.elapsedSeconds() - startTime > 2 && pixelCount == RRAutoDrive.PixelCount.ZERO)
                {
                    robotState = State.COLLECTION_CYCLE_1_WIGGLE;
                    drive.followTrajectorySequenceAsync(trajWiggleNoPixels);
                }
            }
            if (robotState == State.COLLECTION_CYCLE_1_PIXEL_2)
            {
                if (pixelCount == RRAutoDrive.PixelCount.TWO) {
                    robotState = State.DEPOSIT_CYCLE_1_INIT;
                }

                if (drive.elapsedSeconds() - startTime > 1)
                    robotState = State.DEPOSIT_CYCLE_1_INIT;
            }
            if (robotState == State.DEPOSIT_CYCLE_1_INIT)
            {
                Pose2d drivePose;
                if (wiggle == Wiggle.LEFT)
                    drivePose = trajWiggleLeft.end();
                else if (wiggle == Wiggle.RIGHT)
                    drivePose = trajWiggleRight.end();
                else
                    drivePose = trajWiggleNoPixels.end();
                trajSeqStackDrive = drive.trajectorySequenceBuilder(drivePose)
                        .addDisplacementMarker(0, () ->
                        {
                            drive.activateIntake();
                        })
                        .addDisplacementMarker(35, () ->
                        {
                            drive.activateLeftClaw();
                            drive.activateRightClaw();
                            drive.reverseIntake();
                        })
                        .addDisplacementMarker(42, () ->
                        {
                            drive.deactivateIntake();
                        })
                        .addDisplacementMarker(50, () ->
                        {
                            drive.activateLift(liftMotorTicks);
                            drive.initArm();
                        })
                        .addDisplacementMarker(70, () ->
                        {
                            drive.activateArm();
                        })
                        .lineToSplineHeading(new Pose2d(-10, 11, Math.toRadians(180)), velConPixel, accConPixel)
                        .lineToConstantHeading(new Vector2d(56, 11), velConPixel, accConPixel)
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
                        .UNSTABLE_addTemporalMarkerOffset(0.5, () ->
                        {
                            drive.deactivateArm();
                            robotState = State.FIRST_PATHS_AGAIN_INIT;
                        })
                        .build();
                robotState = State.DEPOSIT_CYCLE_1;
                drive.followTrajectorySequenceAsync(trajSeqStackDrive);
            }

            drive.update();
            updateTelemetry();
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

    private void updateTelemetry()
    {
        telemetry.addData("robot state: ", robotState);
        telemetry.addData("pixelCount: ", robotState);
        telemetry.update();
    }
}
