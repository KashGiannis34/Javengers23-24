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
@Autonomous (name = "AutoBlueBackdrop_GREEN_LEMONS_COMPLEX_BACKDROP")
public class AutoBlueBackdrop_GREEN_LEMONS_COMPLEX_BACKDROP extends LinearOpMode {
    RRAutoDrive drive;
    String curAlliance = "blue";

    private TeamElementSubsystem teamElementDetection;

    private int liftMotorTicks = 400;

    enum State
    {
        PATH_RUNNING,
        STACK_PATH,
        INTAKE,
        DEPOSIT_1,
        DEPOSIT_2
    }

    State robotState = State.PATH_RUNNING;

    double startTime;

    SplitAveragePipeline.ZONE zone = SplitAveragePipeline.ZONE.RIGHT;
    public TrajectorySequence trajSeqCenter, trajSeqLeft, trajSeqRight, trajSeqStack, trajSeqDeposit, trajSeqDeposit2;

    Vector2d parkVector = new Vector2d(50, 57.75);
    Vector2d parkVector2 = new Vector2d(60, 57.75);
    TrajectoryVelocityConstraint velConPixel = SampleMecanumDrive.getVelocityConstraint(42, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    TrajectoryAccelerationConstraint accConPixel = SampleMecanumDrive.getAccelerationConstraint(30);

    TrajectoryVelocityConstraint velConPark = SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    TrajectoryAccelerationConstraint accConPark = SampleMecanumDrive.getAccelerationConstraint(20);

    public void runOpMode()
    {
        initialize();

        teamElementDetection = new TeamElementSubsystem(hardwareMap, SplitAveragePipeline.ZONE_VIEW.LEFT);
        while (!isStarted() && !isStopRequested())
        {
            zone = teamElementDetection.getElementZoneValue(telemetry);

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
                    .lineToConstantHeading(new Vector2d(18, 32), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(0, () ->
                    {
                        drive.activateArm();
                    })
                    .lineToLinearHeading(new Pose2d(55.5, 32, Math.toRadians(180)), velConPixel, accConPixel)
                    .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .waitSeconds(.75)
                    .splineToConstantHeading(new Vector2d(parkVector.getX(), 35), Math.toRadians(90), velConPixel, accConPixel)
                    .UNSTABLE_addTemporalMarkerOffset(1, () ->
                    {
                        drive.restArmAuto();
                    })
                    .lineTo(parkVector, velConPixel, accConPixel)
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->
                    {
                        drive.activateIntakeServo();
                        drive.restArmAuto();
                        drive.deactivateLift();
                        robotState = State.STACK_PATH;
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
                    .lineToLinearHeading(new Pose2d(55.5, 28, Math.toRadians(180)), velConPixel, accConPixel)
                    .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .waitSeconds(.75)
                    .splineToConstantHeading(new Vector2d(parkVector.getX(), 30), Math.toRadians(90), velConPixel, accConPixel)
                    .UNSTABLE_addTemporalMarkerOffset(1, () ->
                    {
                        drive.restArmAuto();
                    })
                    .lineTo(parkVector, velConPixel, accConPixel)
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->
                    {
                        drive.activateIntakeServo();
                        drive.deactivateLift();
                        robotState = State.STACK_PATH;
                    })
                    .build();
            trajSeqLeft = drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(() ->
                    {
                        drive.initArm();
                        drive.deactivateIntakeServo();
                        drive.activateLift(liftMotorTicks);
                    })
                    .lineToConstantHeading(new Vector2d(24.5, 42), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(0, () ->
                    {
                        drive.activateArm();
                    })
                    .lineToLinearHeading(new Pose2d(55.5, 39.75, Math.toRadians(180)), velConPixel, accConPixel)
                    .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .waitSeconds(.75)
                    .splineToConstantHeading(new Vector2d(parkVector.getX(), 41.5), Math.toRadians(90), velConPixel, accConPixel)
                    .UNSTABLE_addTemporalMarkerOffset(1, () ->
                    {
                        drive.restArmAuto();
                    })
                    .lineTo(parkVector, velConPixel, accConPixel)
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->
                    {
                        drive.activateIntakeServo();
                        drive.restArmAuto();
                        drive.deactivateLift();
                        robotState = State.STACK_PATH;
                    })
                    .build();
        }
        else {
            // do nothing!
        }

        waitForStart();
        drive.resetRuntime();
        if (isStopRequested()) return;
        Pose2d cyclePose;
        if (zone == SplitAveragePipeline.ZONE.LEFT) {
            drive.followTrajectorySequenceAsync(trajSeqLeft);
            cyclePose = trajSeqLeft.end();
        }
        else if (zone == SplitAveragePipeline.ZONE.CENTER) {
            drive.followTrajectorySequenceAsync(trajSeqCenter);
            cyclePose = trajSeqCenter.end();
        }
        else {
            drive.followTrajectorySequenceAsync(trajSeqRight);
            cyclePose = trajSeqRight.end();
        }

        while (opModeIsActive() && !isStopRequested())
        {
            if (robotState == State.STACK_PATH)
            {
                trajSeqStack = drive.trajectorySequenceBuilder(cyclePose)
                        .addDisplacementMarker(10, () -> {
                            drive.deactivateArm();
                        })
                        .addDisplacementMarker(15, () -> {
                            drive.deactivateIntakeServo();
                        })
                        .lineToConstantHeading(new Vector2d(-40, 57.75), velConPixel, accConPixel)
                        .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                            drive.activateIntake();
                            drive.activateIntakeServoTwo();
                        })
                        .splineToConstantHeading(new Vector2d(-56.7, 50), Math.toRadians(270), velConPark, accConPark)
                        .lineToSplineHeading(new Pose2d(-56.7, 31, Math.toRadians(195)), velConPark, accConPark)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            robotState = State.INTAKE;
                            startTime = drive.elapsedSeconds();
                        })
                        .build();
                robotState = State.PATH_RUNNING;
                drive.followTrajectorySequenceAsync(trajSeqStack);
            }
            if (robotState == State.INTAKE)
            {
                if (drive.elapsedSeconds() - startTime > 2 || drive.getPixelCount() == RRAutoDrive.PixelCount.TWO)
                {
                    if (drive.elapsedSeconds() > 20)
                        robotState = State.DEPOSIT_1;
                    else
                        robotState = State.DEPOSIT_2;
                }
            }
            if (robotState == State.DEPOSIT_1)
            {
                trajSeqDeposit = drive.trajectorySequenceBuilder(trajSeqStack.end())
                        .addDisplacementMarker(40, () -> {
                            drive.activateRightClaw();
                            drive.activateLeftClaw();
                        })
                        .addDisplacementMarker(45, () -> {
                            drive.reverseIntake();
                        })
                        .addDisplacementMarker(52, () -> {
                            drive.deactivateIntake();
                        })
                        .lineToSplineHeading(new Pose2d((trajSeqStack.end().getX()-34.0)/2, (58.0+trajSeqStack.end().getY())/2, Math.toRadians(160)), velConPixel, accConPixel)
                        .lineToSplineHeading(new Pose2d(-34, 58, Math.toRadians(180)), velConPixel, accConPixel)
                        .UNSTABLE_addDisplacementMarkerOffset(60, () ->
                        {
                            drive.activateIntakeServo();
                            drive.activateLift(liftMotorTicks);
                            drive.initArm();
                        })
                        .UNSTABLE_addDisplacementMarkerOffset(74, () ->
                        {
                            drive.deactivateIntakeServo();
                            drive.activateArm();
                        })
                        .lineToConstantHeading(new Vector2d(50, 58), velConPixel, accConPixel)
                        .UNSTABLE_addTemporalMarkerOffset(0, () ->
                        {
                            drive.deactivateLeftClaw();
                            drive.deactivateRightClaw();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                            drive.activateIntakeServo();
                            drive.restArmAuto();
                            drive.deactivateLift();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1.25, () -> {
                            drive.deactivateIntakeServo();
                            drive.deactivateArm();
                        })
                        .waitSeconds(1.5)
                        .build();
                robotState = State.PATH_RUNNING;
                drive.followTrajectorySequenceAsync(trajSeqDeposit);
            }
            if (robotState == State.DEPOSIT_2)
            {
                trajSeqDeposit2 = drive.trajectorySequenceBuilder(trajSeqStack.end())
                        .addDisplacementMarker(40, () -> {
                            drive.activateRightClaw();
                            drive.activateLeftClaw();
                        })
                        .addDisplacementMarker(45, () -> {
                            drive.reverseIntake();
                        })
                        .addDisplacementMarker(52, () -> {
                            drive.deactivateIntake();
                        })
                        .lineToSplineHeading(new Pose2d((trajSeqStack.end().getX()-34.0)/2, (58.0+trajSeqStack.end().getY())/2, Math.toRadians(160)), velConPixel, accConPixel)
                        .lineToSplineHeading(new Pose2d(-34, 58, Math.toRadians(180)), velConPixel, accConPixel)
                        .UNSTABLE_addDisplacementMarkerOffset(60, () ->
                        {
                            drive.activateIntakeServo();
                            drive.activateLift(642);
                            drive.initArm();
                        })
                        .UNSTABLE_addDisplacementMarkerOffset(74, () ->
                        {
                            drive.deactivateIntakeServo();
                            drive.activateArm();
                        })
                        .lineToConstantHeading(new Vector2d(20, 58), velConPixel, accConPixel)
                        .splineToConstantHeading(new Vector2d(50, 38), Math.toRadians(90), velConPixel, accConPixel)
                        .lineToConstantHeading(new Vector2d(55.75, 38), velConPixel, accConPixel)
                        .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                            drive.deactivateRightClaw();
                            drive.deactivateLeftClaw();
                        })
                        .waitSeconds(.75)
                        .splineToConstantHeading(new Vector2d(parkVector.getX(), 38), Math.toRadians(90), velConPixel, accConPixel)
                        .UNSTABLE_addTemporalMarkerOffset(1, () ->
                        {
                            drive.restArm();
                        })
                        .lineTo(parkVector, velConPixel, accConPixel)
                        .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                            drive.deactivateLift();
                        })
                        .lineTo(parkVector2, velConPixel, accConPixel)
                        .build();
                robotState = State.PATH_RUNNING;
                drive.followTrajectorySequenceAsync(trajSeqDeposit2);
            }
            telemetry.addData("elapsed time: ", drive.elapsedSeconds());
            telemetry.update();
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
