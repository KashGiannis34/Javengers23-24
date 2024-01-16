package org.firstinspires.ftc.teamcode.ftc18305.auto.old;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ftc18305.auto.RRAutoDrive;

@Disabled
@Autonomous
public class Auto_Blue_NEW extends LinearOpMode {
    private RRAutoDrive drive;
    private static int elementPosition = 0;

    private int armTicksQuarter = 200;

    // RPM = 435; TICKS_PER_REV = 384.5; Gear Ratio = 2;
    int TICKS_FOR_ONE = 769; // Ticks for 1 rotation is 384.5 * 2 = 769;
    int TICKS_FOR_TWO = 1538; // Ticks for 2 rotation is 384.5 * 4 = 1538;
    int TICKS_FOR_THREE = 2307; // Ticks for 3 rotation is 384.5 * 6 = 2307;
    int TICKS_FOR_FOURADJUSTED = 3100; // Ticks for 4 rotation is 384.5 * 8 = 3076;
    int TICKS_FOR_FOURADJUSTED2 = 3200; // Ticks for 4 rotation is 384.5 * 8 = 3076;
    int TICKS_FOR_FIVE = 3845; // Ticks for 5 rotation is 384.5 * 10 = 3845;
    private int liftMotorTicks = TICKS_FOR_FOURADJUSTED;

    Trajectory trajLow1, trajLow2, trajLow1back,trajLow2back, trajLow3, trajLow4, trajLow5, trajLow6, trajLow7, trajLow8, trajLow9, trajLow10, trajLow11, trajLow12, trajLow13, trajLow14;
    Trajectory trajMid1, trajMid2, trajMid1back,trajMid2back, trajMid3, trajMid4, trajMid5, trajMid6, trajMid7, trajMid8, trajMid9, trajMid10, trajMid11, trajMid12, trajMid13, trajMid14;
    Trajectory trajHigh1, trajHigh2, trajHigh1back,trajHigh2back, trajHigh3, trajHigh4, trajHigh5, trajHigh6, trajHigh7, trajHigh8, trajHigh9, trajHigh10, trajHigh11, trajHigh12, trajHigh13, trajHigh14;

    public void runOpMode() {
        initialize();

        Pose2d startPose = new Pose2d();
        Pose2d nextPose = startPose;
        Pose2d nextPoseCycleAuto;

        drive.setPoseEstimate(startPose);
/*
// ***** Trajectories for LOW Starts *****
        trajLow1 = drive.trajectoryBuilder(startPose, true)
                .addDisplacementMarker(() -> {
                    drive.activateDumpServoHalf();
                })
                .splineTo(new Vector2d(-16,-20.5), Math.toRadians(-105))
                .addTemporalMarker(1.2, () -> {
                    drive.activateDumpServo();
                })
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajLow2))
                .build();
        nextPoseCycleAuto = trajLow1.end();

        trajLow2 = drive.trajectoryBuilder(nextPoseCycleAuto)
                .splineTo(new Vector2d(-8,0), Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(-10,-3, Math.toRadians(0)), Math.toRadians(90))
                .addTemporalMarker(0, () -> {
                    drive.deactivateDumpServo();
                    drive.startIntake(1.0);
                })
                .splineToConstantHeading(
                        new Vector2d(0,6), Math.toRadians(0))
                .splineToConstantHeading(
                        new Vector2d(32,10), Math.toRadians(0))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajLow1back))
                .build();
        nextPoseCycleAuto = trajLow2.end();

        trajLow1back = drive.trajectoryBuilder(nextPoseCycleAuto)
                .addTemporalMarker(0.5, () -> {
                    drive.reverseIntake(1.0);
                    drive.activateLift(liftMotorTicks);
                    drive.activateDumpServoHalf();
                })
                .lineToConstantHeading(
                        new Vector2d(0,20))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajLow3))
                .build();
        nextPoseCycleAuto = trajLow1back.end();

        trajLow3 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
                .addTemporalMarker(1.1, () -> {
                    drive.activateDumpServo();
                })
                .splineTo(new Vector2d(-19,2), Math.toRadians(-105))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajLow4))
                .build();
        nextPoseCycleAuto = trajLow3.end();
//
        trajLow4 = drive.trajectoryBuilder(nextPoseCycleAuto)
                .splineTo(new Vector2d(-15,20), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    drive.deactivateLift();
                    drive.deactivateDumpServo();
                    drive.startIntake(1.0);
                })
                .splineToConstantHeading(
                        new Vector2d(-8,26), Math.toRadians(0))
                .splineToConstantHeading(
                        new Vector2d(32,28), Math.toRadians(0))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajLow2back))
                .build();
        nextPoseCycleAuto = trajLow4.end();

        trajLow2back = drive.trajectoryBuilder(nextPoseCycleAuto)
                .addTemporalMarker(0.4, () -> {
                    drive.reverseIntake(1.0);
                    drive.activateLift(TICKS_FOR_FOURADJUSTED2);
                    drive.activateDumpServoHalf();
                })
                .lineToConstantHeading(
                        new Vector2d(0,30))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajLow5))
                .build();
        nextPoseCycleAuto = trajLow2back.end();

        trajLow5 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
                .addTemporalMarker(1.3, () -> {
                    drive.activateDumpServo();
                })
                .splineTo(new Vector2d(-20,11), Math.toRadians(-105))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajLow6))
                .build();
        nextPoseCycleAuto = trajLow5.end();

        trajLow6 = drive.trajectoryBuilder(nextPoseCycleAuto)
                .splineTo(new Vector2d(-15,30), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    drive.deactivateLift();
                    drive.deactivateDumpServo();
                    drive.startIntake(1.0);
                })
                .splineToConstantHeading(
                        new Vector2d(-8,36), Math.toRadians(0))
                .splineToConstantHeading(
                        new Vector2d(33,38), Math.toRadians(0))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajLow7))
                .build();
        nextPoseCycleAuto = trajLow6.end();

        trajLow7 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
                .addTemporalMarker(0.6, () -> {
                    drive.reverseIntake(1.0);
                    drive.activateLift(liftMotorTicks);
                    drive.activateDumpServoHalf();
                })
                .lineToConstantHeading(
                        new Vector2d(0,40))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajLow8))
                .build();
        nextPoseCycleAuto = trajLow7.end();

        trajLow8 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
                .addTemporalMarker(1.3, () -> {
                    drive.activateDumpServo();
                })
                .splineTo(new Vector2d(-24 ,22), Math.toRadians(-105))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajLow9))
                .build();
        nextPoseCycleAuto = trajLow8.end();

        trajLow9 = drive.trajectoryBuilder(nextPoseCycleAuto)
                .splineTo(new Vector2d(-15,40), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    drive.deactivateLift();
                    drive.deactivateDumpServo();
                    drive.startIntake(1.0);
                })
                .splineToConstantHeading(
                        new Vector2d(-8,46), Math.toRadians(0))
                .splineToConstantHeading(
                        new Vector2d(33,48), Math.toRadians(0))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajLow10))
                .build();
        nextPoseCycleAuto = trajLow9.end();

        trajLow10 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
                .addTemporalMarker(0.6, () -> {
                    drive.reverseIntake(1.0);
                    drive.activateLift(liftMotorTicks);
                    drive.activateDumpServoHalf();
                })
                .lineToConstantHeading(
                        new Vector2d(0,50))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajLow11))
                .build();
        nextPoseCycleAuto = trajLow10.end();

        trajLow11 = drive.trajectoryBuilder(nextPoseCycleAuto,true)
                .addTemporalMarker(1.3, () -> {
                    drive.activateDumpServo();
                })
                .splineTo(new Vector2d(-24 ,32), Math.toRadians(-105))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajLow12))
                .build();
        nextPoseCycleAuto = trajLow11.end();

        trajLow12 = drive.trajectoryBuilder(nextPoseCycleAuto)
                .splineTo(new Vector2d(-15,50), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    drive.deactivateLift();
                    drive.deactivateDumpServo();
                    drive.startIntake(1.0);
                })
                .splineToConstantHeading(
                        new Vector2d(-8,56), Math.toRadians(0))
                .splineToConstantHeading(
                        new Vector2d(25,58), Math.toRadians(0))
                .build();
        nextPoseCycleAuto = trajLow12.end();
// ******* LOW Ends *******
// ******** Trajectories for MID Starts ********
        trajMid1 = drive.trajectoryBuilder(startPose, true)
                .addDisplacementMarker(() -> {
                    drive.activateDumpServoHalf();
                    drive.activateLift(TICKS_FOR_TWO);
                })
                .splineTo(new Vector2d(-16,-20.5), Math.toRadians(-105))
                .addTemporalMarker(1.2, () -> {
                    drive.activateDumpServo();
                })
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajMid2))
                .build();
        nextPoseCycleAuto = trajMid1.end();

        trajMid2 = drive.trajectoryBuilder(nextPoseCycleAuto)
                .splineTo(new Vector2d(-8,0), Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(-10,-3, Math.toRadians(0)), Math.toRadians(90))
                .addTemporalMarker(0, () -> {
                    drive.deactivateLift();
                    drive.deactivateDumpServo();
                    drive.startIntake(1.0);
                })
                .splineToConstantHeading(
                        new Vector2d(0,6), Math.toRadians(0))
                .splineToConstantHeading(
                        new Vector2d(32,10), Math.toRadians(0))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajMid1back))
                .build();
        nextPoseCycleAuto = trajMid2.end();

        trajMid1back = drive.trajectoryBuilder(nextPoseCycleAuto)
                .addTemporalMarker(0.5, () -> {
                    drive.reverseIntake(1.0);
                    drive.activateLift(liftMotorTicks);
                    drive.activateDumpServoHalf();
                })
                .lineToConstantHeading(
                        new Vector2d(0,20))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajMid3))
                .build();
        nextPoseCycleAuto = trajMid1back.end();

        trajMid3 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
                .addTemporalMarker(1.1, () -> {
                    drive.activateDumpServo();
                })
                .splineTo(new Vector2d(-19,2), Math.toRadians(-105))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajMid4))
                .build();
        nextPoseCycleAuto = trajMid3.end();
//
        trajMid4 = drive.trajectoryBuilder(nextPoseCycleAuto)
                .splineTo(new Vector2d(-15,20), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    drive.deactivateLift();
                    drive.deactivateDumpServo();
                    drive.startIntake(1.0);
                })
                .splineToConstantHeading(
                        new Vector2d(-8,26), Math.toRadians(0))
                .splineToConstantHeading(
                        new Vector2d(32,28), Math.toRadians(0))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajMid2back))
                .build();
        nextPoseCycleAuto = trajMid4.end();

        trajMid2back = drive.trajectoryBuilder(nextPoseCycleAuto)
                .addTemporalMarker(0.4, () -> {
                    drive.reverseIntake(1.0);
                    drive.activateLift(TICKS_FOR_FOURADJUSTED2);
                    drive.activateDumpServoHalf();
                })
                .lineToConstantHeading(
                        new Vector2d(0,30))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajMid5))
                .build();
        nextPoseCycleAuto = trajMid2back.end();

        trajMid5 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
                .addTemporalMarker(1.3, () -> {
                    drive.activateDumpServo();
                })
                .splineTo(new Vector2d(-20,11), Math.toRadians(-105))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajMid6))
                .build();
        nextPoseCycleAuto = trajMid5.end();

        trajMid6 = drive.trajectoryBuilder(nextPoseCycleAuto)
                .splineTo(new Vector2d(-15,30), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    drive.deactivateLift();
                    drive.deactivateDumpServo();
                    drive.startIntake(1.0);
                })
                .splineToConstantHeading(
                        new Vector2d(-8,36), Math.toRadians(0))
                .splineToConstantHeading(
                        new Vector2d(33,38), Math.toRadians(0))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajMid7))
                .build();
        nextPoseCycleAuto = trajMid6.end();

        trajMid7 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
                .addTemporalMarker(0.6, () -> {
                    drive.reverseIntake(1.0);
                    drive.activateLift(liftMotorTicks);
                    drive.activateDumpServoHalf();
                })
                .lineToConstantHeading(
                        new Vector2d(0,40))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajMid8))
                .build();
        nextPoseCycleAuto = trajMid7.end();

        trajMid8 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
                .addTemporalMarker(1.3, () -> {
                    drive.activateDumpServo();
                })
                .splineTo(new Vector2d(-24 ,22), Math.toRadians(-105))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajMid9))
                .build();
        nextPoseCycleAuto = trajMid8.end();

        trajMid9 = drive.trajectoryBuilder(nextPoseCycleAuto)
                .splineTo(new Vector2d(-15,40), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    drive.deactivateLift();
                    drive.deactivateDumpServo();
                    drive.startIntake(1.0);
                })
                .splineToConstantHeading(
                        new Vector2d(-8,46), Math.toRadians(0))
                .splineToConstantHeading(
                        new Vector2d(33,48), Math.toRadians(0))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajMid10))
                .build();
        nextPoseCycleAuto = trajMid9.end();

        trajMid10 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
                .addTemporalMarker(0.6, () -> {
                    drive.reverseIntake(1.0);
                    drive.activateLift(liftMotorTicks);
                    drive.activateDumpServoHalf();
                })
                .lineToConstantHeading(
                        new Vector2d(0,50))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajMid11))
                .build();
        nextPoseCycleAuto = trajMid10.end();

        trajMid11 = drive.trajectoryBuilder(nextPoseCycleAuto,true)
                .addTemporalMarker(1.3, () -> {
                    drive.activateDumpServo();
                })
                .splineTo(new Vector2d(-24 ,32), Math.toRadians(-105))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajMid12))
                .build();
        nextPoseCycleAuto = trajMid11.end();

        trajMid12 = drive.trajectoryBuilder(nextPoseCycleAuto)
                .splineTo(new Vector2d(-15,50), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    drive.deactivateLift();
                    drive.deactivateDumpServo();
                    drive.startIntake(1.0);
                })
                .splineToConstantHeading(
                        new Vector2d(-8,56), Math.toRadians(0))
                .splineToConstantHeading(
                        new Vector2d(25,58), Math.toRadians(0))
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(traj13))
                .build();
        nextPoseCycleAuto = trajMid12.end();

// ******** MID Ends ********
// ******** Trajectories for HIGH Starts ********
        trajHigh1 = drive.trajectoryBuilder(startPose, true)
                .addDisplacementMarker(() -> {
                    drive.activateDumpServoHalf();
                    drive.activateLift(liftMotorTicks);
                })
                .splineTo(new Vector2d(-16,-20.5), Math.toRadians(-105))
                .addTemporalMarker(1.2, () -> {
                    drive.activateDumpServo();
                })
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajHigh2))
                .build();
        nextPoseCycleAuto = trajHigh1.end();

        trajHigh2 = drive.trajectoryBuilder(nextPoseCycleAuto)
                .splineTo(new Vector2d(-8,0), Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(-10,-3, Math.toRadians(0)), Math.toRadians(90))
                .addTemporalMarker(0, () -> {
                    drive.deactivateLift();
                    drive.deactivateDumpServo();
                    drive.startIntake(1.0);
                })
                .splineToConstantHeading(
                        new Vector2d(0,6), Math.toRadians(0))
                .splineToConstantHeading(
                        new Vector2d(32,10), Math.toRadians(0))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajHigh1back))
                .build();
        nextPoseCycleAuto = trajHigh2.end();

        trajHigh1back = drive.trajectoryBuilder(nextPoseCycleAuto)
                .addTemporalMarker(0.5, () -> {
                    drive.reverseIntake(1.0);
                    drive.activateLift(liftMotorTicks);
                    drive.activateDumpServoHalf();
                })
                .lineToConstantHeading(
                        new Vector2d(0,20))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajHigh3))
                .build();
        nextPoseCycleAuto = trajHigh1back.end();

        trajHigh3 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
                .addTemporalMarker(1.1, () -> {
                    drive.activateDumpServo();
                })
                .splineTo(new Vector2d(-19,2), Math.toRadians(-105))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajHigh4))
                .build();
        nextPoseCycleAuto = trajHigh3.end();
//
        trajHigh4 = drive.trajectoryBuilder(nextPoseCycleAuto)
                .splineTo(new Vector2d(-15,20), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    drive.deactivateLift();
                    drive.deactivateDumpServo();
                    drive.startIntake(1.0);
                })
                .splineToConstantHeading(
                        new Vector2d(-8,26), Math.toRadians(0))
                .splineToConstantHeading(
                        new Vector2d(32,28), Math.toRadians(0))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajHigh2back))
                .build();
        nextPoseCycleAuto = trajHigh4.end();

        trajHigh2back = drive.trajectoryBuilder(nextPoseCycleAuto)
                .addTemporalMarker(0.4, () -> {
                    drive.reverseIntake(1.0);
                    drive.activateLift(TICKS_FOR_FOURADJUSTED2);
                    drive.activateDumpServoHalf();
                })
                .lineToConstantHeading(
                        new Vector2d(0,30))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajHigh5))
                .build();
        nextPoseCycleAuto = trajHigh2back.end();

        trajHigh5 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
                .addTemporalMarker(1.3, () -> {
                    drive.activateDumpServo();
                })
                .splineTo(new Vector2d(-20,11), Math.toRadians(-105))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajHigh6))
                .build();
        nextPoseCycleAuto = trajHigh5.end();

        trajHigh6 = drive.trajectoryBuilder(nextPoseCycleAuto)
                .splineTo(new Vector2d(-15,30), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    drive.deactivateLift();
                    drive.deactivateDumpServo();
                    drive.startIntake(1.0);
                })
                .splineToConstantHeading(
                        new Vector2d(-8,36), Math.toRadians(0))
                .splineToConstantHeading(
                        new Vector2d(33,38), Math.toRadians(0))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajHigh7))
                .build();
        nextPoseCycleAuto = trajHigh6.end();

        trajHigh7 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
                .addTemporalMarker(0.6, () -> {
                    drive.reverseIntake(1.0);
                    drive.activateLift(liftMotorTicks);
                    drive.activateDumpServoHalf();
                })
                .lineToConstantHeading(
                        new Vector2d(0,40))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajHigh8))
                .build();
        nextPoseCycleAuto = trajHigh7.end();

        trajHigh8 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
                .addTemporalMarker(1.3, () -> {
                    drive.activateDumpServo();
                })
                .splineTo(new Vector2d(-24 ,22), Math.toRadians(-105))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajHigh9))
                .build();
        nextPoseCycleAuto = trajHigh8.end();

        trajHigh9 = drive.trajectoryBuilder(nextPoseCycleAuto)
                .splineTo(new Vector2d(-15,40), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    drive.deactivateLift();
                    drive.deactivateDumpServo();
                    drive.startIntake(1.0);
                })
                .splineToConstantHeading(
                        new Vector2d(-8,46), Math.toRadians(0))
                .splineToConstantHeading(
                        new Vector2d(33,48), Math.toRadians(0))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajHigh10))
                .build();
        nextPoseCycleAuto = trajHigh9.end();

        trajHigh10 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
                .addTemporalMarker(0.6, () -> {
                    drive.reverseIntake(1.0);
                    drive.activateLift(liftMotorTicks);
                    drive.activateDumpServoHalf();
                })
                .lineToConstantHeading(
                        new Vector2d(0,50))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajHigh11))
                .build();
        nextPoseCycleAuto = trajHigh10.end();

        trajHigh11 = drive.trajectoryBuilder(nextPoseCycleAuto,true)
                .addTemporalMarker(1.3, () -> {
                    drive.activateDumpServo();
                })
                .splineTo(new Vector2d(-24 ,32), Math.toRadians(-105))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajHigh12))
                .build();
        nextPoseCycleAuto = trajHigh11.end();

        trajHigh12 = drive.trajectoryBuilder(nextPoseCycleAuto)
                .splineTo(new Vector2d(-15,50), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    drive.deactivateLift();
                    drive.deactivateDumpServo();
                    drive.startIntake(1.0);
                })
                .splineToConstantHeading(
                        new Vector2d(-8,56), Math.toRadians(0))
                .splineToConstantHeading(
                        new Vector2d(25,58), Math.toRadians(0))
                .build();
        nextPoseCycleAuto = trajHigh12.end();
// ******** HIGH Ends ********

        waitForStart();

        if (isStopRequested()) return;

        elementPosition = BarcodeDetectionBluePipeline.position;
        telemetry.addData("Position: ", elementPosition);
        telemetry.update();

        if(elementPosition == 1) {
            drive.followTrajectoryAsync(trajLow1);
            //drive.activateLift(TICKS_FOR_ONE);
        } else if(elementPosition == 2) {
            drive.followTrajectoryAsync(trajMid1);
            //drive.activateLift(TICKS_FOR_TWO);
        } else {
            //position = 3
            drive.followTrajectoryAsync(trajHigh1);
            //drive.activateLift(TICKS_FOR_THREE);
        }

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
        }

 */
    }

    private void initialize() {
        drive = new RRAutoDrive(hardwareMap);
//        drive.startCamera();
//        BarcodeDetectionBluePipeline.setRegionValues(6, 72, 5);
        telemetry.addData("Status:", "initialize() - Robot and Camera are initialized!");
        telemetry.update();
    }

}