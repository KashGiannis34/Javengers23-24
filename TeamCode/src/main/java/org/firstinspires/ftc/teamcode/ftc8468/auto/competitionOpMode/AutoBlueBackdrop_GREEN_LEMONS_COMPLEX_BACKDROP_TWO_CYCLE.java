package org.firstinspires.ftc.teamcode.ftc8468.auto.competitionOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ftc8468.auto.RRAutoDrive;
import org.firstinspires.ftc.teamcode.ftc8468.auto.pipelines.SplitAveragePipeline;
import org.firstinspires.ftc.teamcode.ftc8468.auto.pipelines.TeamElementSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous (name = "AutoBlueBackdrop_GREEN_LEMONS_COMPLEX_BACKDROP_TWO_CYCLE")
public class AutoBlueBackdrop_GREEN_LEMONS_COMPLEX_BACKDROP_TWO_CYCLE extends LinearOpMode {
    RRAutoDrive drive;
    String curAlliance = "blue";

    boolean secondCycle = false;

    private TeamElementSubsystem teamElementDetection;

    private int liftMotorTicks = 400;

    enum State
    {
        PATH_RUNNING,
        STACK_PATH,
        INTAKE,
        DEPOSIT_1,
        DEPOSIT_2,
        PARK
    }

    State robotState = State.PATH_RUNNING;

    double startTime;

    State depositState;

    SplitAveragePipeline.ZONE zone = SplitAveragePipeline.ZONE.RIGHT;
    public TrajectorySequence trajSeqCenter, trajSeqLeft, trajSeqRight, trajSeqStack, trajSeqDeposit, trajSeqDeposit2, trajSeqPark;

    Vector2d parkVector = new Vector2d(50, 54.75);
    Vector2d parkVector2 = new Vector2d(60, 57.75);
    TrajectoryVelocityConstraint velConPixel = RRAutoDrive.velCon(47);
    TrajectoryAccelerationConstraint accConPixel = RRAutoDrive.accCon(34);

    TrajectoryVelocityConstraint velConPark = RRAutoDrive.velCon(25);
    TrajectoryAccelerationConstraint accConPark = RRAutoDrive.accCon(20);

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
                    .lineToLinearHeading(new Pose2d(55.25, 32, Math.toRadians(180)), velConPixel, accConPixel)
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
                    .lineToLinearHeading(new Pose2d(55.25, 28, Math.toRadians(180)), velConPixel, accConPixel)
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
                    .lineToLinearHeading(new Pose2d(55.25, 39.75, Math.toRadians(180)), velConPixel, accConPixel)
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
        if (zone == SplitAveragePipeline.ZONE.LEFT) {
            drive.followTrajectorySequenceAsync(trajSeqLeft);
        }
        else if (zone == SplitAveragePipeline.ZONE.CENTER) {
            drive.followTrajectorySequenceAsync(trajSeqCenter);
        }
        else {
            drive.followTrajectorySequenceAsync(trajSeqRight);
        }

        while (opModeIsActive() && !isStopRequested())
        {
            if (robotState == State.STACK_PATH)
            {
                Pose2d cyclePose;
                if (secondCycle) {
                    if (depositState == State.DEPOSIT_1) {
                        cyclePose = trajSeqDeposit.end();
                    }
                    else
                    {
                        cyclePose = trajSeqDeposit2.end();
                    }
                }
                else
                    cyclePose = trajSeqCenter.end();

                trajSeqStack = drive.trajectorySequenceBuilder(cyclePose)
                        .addDisplacementMarker(10, () -> {
                            drive.deactivateArm();
                        })
                        .addDisplacementMarker(15, () -> {
                            drive.deactivateIntakeServo();
                        })
                        .lineToConstantHeading(new Vector2d(-40, parkVector.getY()), velConPixel, accConPixel)
                        .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                            drive.activateIntake();
                            if (secondCycle)
                                drive.activateIntakeServoOne();
                            else
                                drive.activateIntakeServoTwo();
                        })
                        .splineToConstantHeading(new Vector2d(-56.25, 50), Math.toRadians(270), velConPark, accConPark)
                        .lineToSplineHeading(new Pose2d(-56.25, (secondCycle ? 27:31), Math.toRadians(195)), velConPark, accConPark)
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
                if (drive.elapsedSeconds() - startTime > 1.5 || drive.getPixelCount() == RRAutoDrive.PixelCount.TWO)
                {
                    if (drive.elapsedSeconds() > 69) {
                        robotState = State.DEPOSIT_1;
                        depositState = State.DEPOSIT_1;
                    }
                    else {
                        robotState = State.DEPOSIT_2;
                        depositState = State.DEPOSIT_2;
                    }
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
                        .lineToSplineHeading(new Pose2d((trajSeqStack.end().getX()-34.0)/2, (parkVector.getY()+trajSeqStack.end().getY())/2, Math.toRadians(160)), RRAutoDrive.velCon(55), RRAutoDrive.accCon(37))
                        .lineToSplineHeading(new Pose2d(-34, parkVector.getY(), Math.toRadians(180)), RRAutoDrive.velCon(55), RRAutoDrive.accCon(37))
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
                        .lineToConstantHeading(parkVector, RRAutoDrive.velCon(55), RRAutoDrive.accCon(37))
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
                            if (drive.elapsedSeconds() < 23)
                            {
                                secondCycle = true;
                                robotState = State.STACK_PATH;
                            }
                            else
                            {
                                robotState = State.PARK;
                            }
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
                        .lineToSplineHeading(new Pose2d((trajSeqStack.end().getX()-34.0)/2, (parkVector.getY()+trajSeqStack.end().getY())/2, Math.toRadians(160)), RRAutoDrive.velCon(47), RRAutoDrive.accCon(37))
                        .lineToSplineHeading(new Pose2d(-34, parkVector.getY(), Math.toRadians(180)), velConPixel, accConPixel)
                        .UNSTABLE_addDisplacementMarkerOffset(60, () ->
                        {
                            drive.activateIntakeServo();
                            drive.activateLift(700);
                            drive.initArm();
                        })
                        .UNSTABLE_addDisplacementMarkerOffset(74, () ->
                        {
                            drive.deactivateIntakeServo();
                            drive.activateArm();
                        })
                        .lineToConstantHeading(new Vector2d(20, parkVector.getY()), RRAutoDrive.velCon(47), RRAutoDrive.accCon(37))
                        .splineToConstantHeading(new Vector2d(50, 34), Math.toRadians(90), RRAutoDrive.velCon(47), RRAutoDrive.accCon(37))
                        .lineToConstantHeading(new Vector2d(54.75, 34), RRAutoDrive.velCon(25), RRAutoDrive.accCon(20))
                        .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                            drive.deactivateRightClaw();
                            drive.deactivateLeftClaw();
                        })
                        .waitSeconds(.75)
                        .splineToConstantHeading(new Vector2d(parkVector.getX(), 34), Math.toRadians(90), velConPixel, accConPixel)
                        .UNSTABLE_addTemporalMarkerOffset(1, () ->
                        {
                            drive.restArm();
                        })
                        .lineToConstantHeading(parkVector, velConPixel, accConPixel)
                        .UNSTABLE_addDisplacementMarkerOffset(0,() -> {
                            drive.activateIntakeServo();
                            drive.deactivateLift();
                            if (drive.elapsedSeconds() < 17)
                            {
                                secondCycle = true;
                                robotState = State.STACK_PATH;
                            }
                        })
                        .build();
                robotState = State.PATH_RUNNING;
                drive.followTrajectorySequenceAsync(trajSeqDeposit2);
            }
            if (robotState == State.PARK)
            {
                trajSeqPark = drive.trajectorySequenceBuilder(trajSeqDeposit.end())
                        .lineToConstantHeading(parkVector2, velConPark, accConPark)
                        .build();
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
