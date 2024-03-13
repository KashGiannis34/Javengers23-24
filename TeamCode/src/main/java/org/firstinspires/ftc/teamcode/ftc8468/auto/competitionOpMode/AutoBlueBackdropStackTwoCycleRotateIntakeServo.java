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
import org.firstinspires.ftc.teamcode.ftc8468.RobotConstants;
import org.firstinspires.ftc.teamcode.ftc8468.auto.RRAutoDrive;
import org.firstinspires.ftc.teamcode.ftc8468.auto.pipelines.SplitAveragePipeline;
import org.firstinspires.ftc.teamcode.ftc8468.auto.pipelines.TeamElementSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous (name = "AutoBlueBackdropStackTwoCycleRotateIntakeServo")
public class AutoBlueBackdropStackTwoCycleRotateIntakeServo extends LinearOpMode {
    RRAutoDrive drive;
    String curAlliance = "blue";

    private TeamElementSubsystem teamElementDetection;

    private int liftMotorTicks = 390;

    enum Wiggle
    {
        NO_PIXELS,
        NONE
    }

    Wiggle wiggle = Wiggle.NONE;

    boolean leftPixelContained = false;
    boolean rightPixelContained = false;

    RRAutoDrive.PixelCount pixelCount;

    SplitAveragePipeline.ZONE zone = SplitAveragePipeline.ZONE.RIGHT;
    public TrajectorySequence trajSeqCenter, trajSeqLeft, trajSeqRight, trajSeqStackDrive, trajWiggleNoPixels, trajSecondCycle;
    TrajectoryVelocityConstraint velConPixel = SampleMecanumDrive.getVelocityConstraint(57, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    TrajectoryAccelerationConstraint accConPixel = SampleMecanumDrive.getAccelerationConstraint(35);

    TrajectoryVelocityConstraint velConBackdrop = SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    TrajectoryAccelerationConstraint accConBackdrop = SampleMecanumDrive.getAccelerationConstraint(25);
    TrajectoryVelocityConstraint velConSpline = SampleMecanumDrive.getVelocityConstraint(42, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    TrajectoryAccelerationConstraint accConSpline = SampleMecanumDrive.getAccelerationConstraint(25);

    enum State
    {
        PATH_RUNNING,
        FIRST_PATHS_COMPLETED,
        SET_TIMER,
        LOWER_INTAKE_SERVO,
        WIGGLE,
        DEPOSIT,
        FIRST_PATHS_AGAIN
    }

    State robotState = State.PATH_RUNNING;
    double startTime = 0;
    double initServoPos = 0;

    boolean secondCycle = false;

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
                    .lineToConstantHeading(new Vector2d(-57.3, 11), velConPixel, accConSpline)
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
                    .lineToConstantHeading(new Vector2d(-57.3, 11), velConPixel, accConSpline)
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
                    .lineToConstantHeading(new Vector2d(-57.3, 11), velConPixel, accConSpline)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        robotState = State.FIRST_PATHS_COMPLETED;
                    })
                    .build();
            trajWiggleNoPixels = drive.trajectorySequenceBuilder(trajSeqCenter.end())
                    .forward(1)
                    .back(1)
                    .turn(Math.toRadians(-15))
                    .turn(Math.toRadians(15))
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->{
                        startTime = drive.elapsedSeconds();
                        robotState = State.DEPOSIT;
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
            if (robotState == State.FIRST_PATHS_AGAIN)
            {
                if (drive.elapsedSeconds() > 22)
                    break;
                trajSecondCycle = drive.trajectorySequenceBuilder(trajSeqStackDrive.end())
                        .addDisplacementMarker(60, () ->
                        {
                            drive.activateIntake();
                            drive.setIntakeServoPosition(RobotConstants.INTAKE_POSITION_THREE);
                        })
                        .lineToConstantHeading(new Vector2d(-57, 10), velConPixel, accConSpline)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            robotState = State.FIRST_PATHS_COMPLETED;
                        })
                        .build();
                robotState = State.PATH_RUNNING;
                drive.followTrajectorySequenceAsync(trajSecondCycle);
            }
            if (robotState == State.FIRST_PATHS_COMPLETED)
            {
                startTime = drive.elapsedSeconds();
                robotState = State.LOWER_INTAKE_SERVO;
                initServoPos = drive.getIntakeServoPosition();
            }
            if (robotState == State.LOWER_INTAKE_SERVO)
            {
                if (drive.getIntakeServoPosition() < (secondCycle ? RobotConstants.INTAKE_POSITION_TELEOP : RobotConstants.INTAKE_POSITION_FOUR)) {
                    drive.setIntakeServoPosition(drive.getIntakeServoPosition() + .0023);
                    sleep(20);
                }
                if (drive.elapsedSeconds() - startTime > 2 || pixelCount == RRAutoDrive.PixelCount.TWO) {
                    robotState = State.DEPOSIT;
                    if (pixelCount == RRAutoDrive.PixelCount.ZERO)
                        robotState = State.WIGGLE;
                }

            }
            if (robotState == State.WIGGLE)
            {
                robotState = State.PATH_RUNNING;
                drive.followTrajectorySequenceAsync(trajWiggleNoPixels);
            }
            if (robotState == State.DEPOSIT)
            {
                Pose2d drivePose;
                if (wiggle == Wiggle.NO_PIXELS)
                    drivePose = trajWiggleNoPixels.end();
                else
                    drivePose = trajSeqCenter.end();
                trajSeqStackDrive = drive.trajectorySequenceBuilder(drivePose)
                        .addDisplacementMarker(0, () ->
                        {
                            drive.activateIntake();
                        })
                        .addDisplacementMarker(15, () ->
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
                        .addDisplacementMarker(80, () ->
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
                            robotState = State.FIRST_PATHS_AGAIN;
                            secondCycle = true;
                        })
                        .build();
                robotState = State.PATH_RUNNING;
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
