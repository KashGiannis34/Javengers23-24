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
@Autonomous (name = "AutoRedBackdrop_GREEN_LEMONS_COMPLEX_BACKSTAGE")
public class AutoRedBackdrop_GREEN_LEMONS_COMPLEX_BACKSTAGE extends LinearOpMode {
    RRAutoDrive drive;
    String curAlliance = "red";

    private TeamElementSubsystem teamElementDetection;

    private int liftMotorTicks = 390;

    enum State
    {
        PATH_RUNNING,
        STACK_PATH,
        INTAKE,
        DEPOSIT
    }

    State robotState = State.PATH_RUNNING;

    double startTime;

    SplitAveragePipeline.ZONE zone = SplitAveragePipeline.ZONE.RIGHT;
    public TrajectorySequence trajSeqCenter, trajSeqLeft, trajSeqRight, trajSeqStack, trajSeqDeposit;

    Vector2d parkVector = new Vector2d(50, -61.5);
    Vector2d parkVector2 = new Vector2d(60, -61.5);
    TrajectoryVelocityConstraint velConPixel = SampleMecanumDrive.getVelocityConstraint(42, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    TrajectoryAccelerationConstraint accConPixel = SampleMecanumDrive.getAccelerationConstraint(30);

    TrajectoryVelocityConstraint velConPark = SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    TrajectoryAccelerationConstraint accConPark = SampleMecanumDrive.getAccelerationConstraint(20);

    public void runOpMode()
    {
        initialize();

        teamElementDetection = new TeamElementSubsystem(hardwareMap, SplitAveragePipeline.ZONE_VIEW.RIGHT);
        while (!isStarted() && !isStopRequested())
        {
            zone = teamElementDetection.getElementZoneValue(telemetry);

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
                    .lineToLinearHeading(new Pose2d(54.75, -36.75, Math.toRadians(180)), velConPixel, accConPixel)
                    .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .waitSeconds(.75)
                    .lineToConstantHeading(new Vector2d(parkVector.getX(), -36.75), velConPixel, accConPixel)
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->
                    {
                        drive.restArmAuto();
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
                    .splineToLinearHeading(new Pose2d(10, -30, Math.toRadians(180)), Math.toRadians(180), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(0, () ->
                    {
                        drive.activateArm();
                    })
                    .lineToLinearHeading(new Pose2d(54.75, -31, Math.toRadians(180)), velConPixel, accConPixel)
                    .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .waitSeconds(.75)
                    .lineToConstantHeading(new Vector2d(parkVector.getX(), -31), velConPixel, accConPixel)
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->
                    {
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
                    .lineTo(new Vector2d(27, -38), velConPixel, accConPixel)
                    .UNSTABLE_addDisplacementMarkerOffset(0, () ->
                    {
                        drive.activateArm();
                    })
                    .lineToLinearHeading(new Pose2d(54.85, -42.85, Math.toRadians(180)), velConPixel, accConPixel)
                    .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                        drive.deactivateRightClaw();
                        drive.deactivateLeftClaw();
                    })
                    .waitSeconds(.75)
                    .lineToConstantHeading(new Vector2d(parkVector.getX(), -42.85), velConPixel, accConPixel)
                    .UNSTABLE_addTemporalMarkerOffset(0, () ->
                    {
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
                        .splineToConstantHeading(new Vector2d(10, -57), Math.toRadians(180), velConPixel, accConPixel)
                        .lineToConstantHeading(new Vector2d(-40, -57), velConPixel, accConPixel)
                        .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                            drive.activateIntake();
                            drive.activateIntakeServoThree();
                        })
                        .splineToConstantHeading(new Vector2d(-57.3, -50), Math.toRadians(90), velConPixel, accConPixel)
                        .lineToSplineHeading(new Pose2d(-57.3, -35, Math.toRadians(165)), velConPark, accConPark)
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
                    robotState = State.DEPOSIT;
                }
            }
            if (robotState == State.DEPOSIT)
            {
                trajSeqDeposit = drive.trajectorySequenceBuilder(trajSeqStack.end())
                        .addDisplacementMarker(0, () -> {
                            drive.activateRightClaw();
                            drive.activateLeftClaw();
                            drive.reverseIntake();
                        })
                        .addDisplacementMarker(20, () -> {
                            drive.deactivateIntake();
                        })
                        .lineToSplineHeading(new Pose2d(-57.3, -50, Math.toRadians(180)), velConPixel, accConPixel)
                        .splineToConstantHeading(new Vector2d(-40, -57), Math.toRadians(0), velConPixel, accConPixel)
                        .UNSTABLE_addDisplacementMarkerOffset(60, () ->
                        {
                            drive.activateLift(liftMotorTicks);
                            drive.initArm();
                        })
                        .UNSTABLE_addDisplacementMarkerOffset(85, () ->
                        {
                            drive.activateArm();
                        })
                        .lineToConstantHeading(new Vector2d(50, -57), velConPixel, accConPixel)
                        .UNSTABLE_addTemporalMarkerOffset(0, () ->
                        {
                            drive.deactivateLeftClaw();
                            drive.deactivateRightClaw();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                            drive.restArmAuto();
                            drive.deactivateLift();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1.25, () -> {
                            drive.deactivateArm();
                        })
                        .waitSeconds(1.5)
                        .build();
                robotState = State.PATH_RUNNING;
                drive.followTrajectorySequenceAsync(trajSeqDeposit);
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
