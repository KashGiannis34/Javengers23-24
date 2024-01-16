package org.firstinspires.ftc.teamcode.ftc18305.auto.old;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ftc18305.auto.RRAutoDrive;

@Disabled
@Autonomous
public class Auto_Blue extends LinearOpMode {
    private RRAutoDrive drive;
    private static int elementPosition = 0;

    private int armTicksQuarter = 200;

    // RPM = 435; TICKS_PER_REV = 384.5; Gear Ratio = 2;
    int TICKS_FOR_ONE = 769; // Ticks for 1 rotation is 384.5 * 2 = 769;
    int TICKS_FOR_TWO = 1538; // Ticks for 2 rotation is 384.5 * 4 = 1538;
    int TICKS_FOR_THREE = 2307; // Ticks for 3 rotation is 384.5 * 6 = 2307;
    int TICKS_FOR_FOURADJUSTED = 3100; // Ticks for 4 rotation is 384.5 * 8 = 3076;
    int TICKS_FOR_FIVE = 3845; // Ticks for 5 rotation is 384.5 * 10 = 3845;
    private int liftMotorTicks = TICKS_FOR_FOURADJUSTED;

    Trajectory trajLow1, trajLow2, trajLow3, trajLow4, trajLow5, trajLow6, trajLow7, trajLow8, trajLow9, trajLow10, trajLow11, trajLow12, trajLow13, trajLow14;
    Trajectory trajMid1, trajMid2, trajMid3, trajMid4, trajMid5, trajMid6, trajMid7, trajMid8, trajMid9, trajMid10, trajMid11, trajMid12, trajMid13, trajMid14;
    Trajectory trajHigh1, trajHigh2, trajHigh3, trajHigh4, trajHigh5, trajHigh6, trajHigh7, trajHigh8, trajHigh9, trajHigh10, trajHigh11, trajHigh12, trajHigh13, trajHigh14;

    public void runOpMode() {
//        initialize();
//
//        Pose2d startPose = new Pose2d();
//        Pose2d nextPose = startPose;
//        Pose2d nextPoseCycleAuto;
//
//        drive.setPoseEstimate(startPose);
//
//        //nextPoseCycleAuto = startPose;
//
//// ***** Trajectories for LOW Starts *****
//        trajLow1 = drive.trajectoryBuilder(startPose, true)
//                .addDisplacementMarker(() -> {
//                    drive.activateDumpServoHalf();
//                })
//                .splineTo(new Vector2d(-14,-24), Math.toRadians(-120))
//                .addTemporalMarker(1.2, () -> {
//                    drive.activateDumpServo();
//                })
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajLow2))
//                .build();
//        nextPoseCycleAuto = trajLow1.end();
//
//        trajLow2 = drive.trajectoryBuilder(nextPoseCycleAuto)
//                .splineToSplineHeading(new Pose2d(-5,2, Math.toRadians(0)), Math.toRadians(0))
//                .addTemporalMarker(0, () -> {
//                    drive.deactivateDumpServo();
//                    drive.startIntake(1.0);
//                })
//                .splineToConstantHeading(
//                        new Vector2d(33,3), Math.toRadians(0),
//                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajLow3))
//                .build();
//        nextPoseCycleAuto = trajLow2.end();
//
//        trajLow3 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
//                .addTemporalMarker(0.7, () -> {
//                    drive.reverseIntake(1.0);
//                })
//                .lineToConstantHeading(new Vector2d(6, 5))
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajLow4))
//                .build();
//        nextPoseCycleAuto = trajLow3.end();
//
//        trajLow4 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
//                .addTemporalMarker(0, () -> {
//                    drive.activateLift(liftMotorTicks);
//                    drive.activateDumpServoHalf();
//                })
//                .splineTo(new Vector2d(-15,-19), Math.toRadians(-120))
//                .addTemporalMarker(1.2, () -> {
//                    drive.activateDumpServo();
//                })
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajLow5))
//                .build();
//        nextPoseCycleAuto = trajLow4.end();
//
//        trajLow5 = drive.trajectoryBuilder(nextPoseCycleAuto)
//                .splineToSplineHeading(new Pose2d(-5,6, Math.toRadians(0)), Math.toRadians(0))
//                .addTemporalMarker(0, () -> {
//                    drive.deactivateLift();
//                    drive.deactivateDumpServo();
//                    drive.startIntake(1.0);
//                })
//                .splineTo(
//                        new Vector2d(36,8), Math.toRadians(0),
//                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajLow6))
//                .build();
//        nextPoseCycleAuto = trajLow5.end();
//
//        trajLow6 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
//                .addTemporalMarker(0.7, () -> {
//                    drive.reverseIntake(1.0);
//                })
//                .lineToConstantHeading(new Vector2d(6, 10))
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajLow7))
//                .build();
//        nextPoseCycleAuto = trajLow6.end();
//
//        trajLow7 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
//                .addTemporalMarker(0, () -> {
//                    drive.activateLift(liftMotorTicks);
//                    drive.activateDumpServoHalf();
//                })
//                .splineTo(new Vector2d(-13,-12), Math.toRadians(-120))
//                .addTemporalMarker(1.2, () -> {
//                    drive.activateDumpServo();
//                })
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajLow8))
//                .build();
//        nextPoseCycleAuto = trajLow7.end();
//
//        trajLow8 = drive.trajectoryBuilder(nextPoseCycleAuto)
//                .splineToSplineHeading(new Pose2d(-5,10, Math.toRadians(0)), Math.toRadians(0))
//                .addTemporalMarker(0, () -> {
//                    drive.deactivateLift();
//                    drive.deactivateDumpServo();
//                    drive.startIntake(1.0);
//                })
//                .splineTo(
//                        new Vector2d(40,13), Math.toRadians(0),
//                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajLow9))
//                .build();
//        nextPoseCycleAuto = trajLow8.end();
//
//        trajLow9 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
//                .addTemporalMarker(0.7, () -> {
//                    drive.reverseIntake(1.0);
//                })
//                .lineToConstantHeading(new Vector2d(0, 17))
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajLow10))
//                .build();
//        nextPoseCycleAuto = trajLow9.end();
//
//        trajLow10 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
//                .addTemporalMarker(0, () -> {
//                    drive.activateLift(liftMotorTicks);
//                    drive.activateDumpServoHalf();
//                })
//                .splineTo(new Vector2d(-17,-6), Math.toRadians(-120))
//                .addTemporalMarker(1.4, () -> {
//                    drive.activateDumpServo();
//                })
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajLow11))
//                .build();
//        nextPoseCycleAuto = trajLow10.end();
//
//        trajLow11 = drive.trajectoryBuilder(nextPoseCycleAuto)
//                .addTemporalMarker(0, () -> {
//                    drive.deactivateLift();
//                    drive.deactivateDumpServo();
//                    drive.startIntake(1.0);
//                })
//                .splineToSplineHeading(new Pose2d(-5,16, Math.toRadians(0)), Math.toRadians(0))
//                .splineTo(
//                        new Vector2d(40,19), Math.toRadians(0),
//                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajLow12))
//                .build();
//        nextPoseCycleAuto = trajLow11.end();
//
//        trajLow12 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
//                .addTemporalMarker(0.7, () -> {
//                    drive.reverseIntake(1.0);
//                })
//                .lineToConstantHeading(new Vector2d(0, 20))
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajLow13))
//                .build();
//        nextPoseCycleAuto = trajLow12.end();
//
//        trajLow13 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
//                .addTemporalMarker(0, () -> {
//                    drive.activateLift(liftMotorTicks);
//                    drive.activateDumpServoHalf();
//                })
//                .splineTo(new Vector2d(-17,0), Math.toRadians(-120))
//                .addTemporalMarker(1.2, () -> {
//                    drive.activateDumpServo();
//                })
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajLow14))
//                .build();
//        nextPoseCycleAuto = trajLow13.end();
//
//        trajLow14 = drive.trajectoryBuilder(nextPoseCycleAuto)
//                .addTemporalMarker(0, () -> {
//                    drive.deactivateLift();
//                    drive.deactivateDumpServo();
//                    drive.startIntake(1.0);
//                })
//                .splineToSplineHeading(new Pose2d(-5,23, Math.toRadians(0)), Math.toRadians(0))
//                .splineTo(new Vector2d(34,25), Math.toRadians(0))
//                .build();
//        nextPoseCycleAuto = trajLow14.end();
//// ******* LOW Ends *******
//// ******** Trajectories for MID Starts ********
//        trajMid1 = drive.trajectoryBuilder(startPose, true)
//                .addDisplacementMarker(() -> {
//                    drive.activateDumpServoHalf();
//                    drive.activateLift(TICKS_FOR_TWO);
//                })
//                .splineTo(new Vector2d(-14,-24), Math.toRadians(-120))
//                .addTemporalMarker(1.2, () -> {
//                    drive.activateDumpServo();
//                })
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajMid2))
//                .build();
//        nextPoseCycleAuto = trajMid1.end();
//
//        trajMid2 = drive.trajectoryBuilder(nextPoseCycleAuto)
//                .splineToSplineHeading(new Pose2d(-5,2, Math.toRadians(0)), Math.toRadians(0))
//                .addTemporalMarker(0, () -> {
//                    drive.deactivateLift();
//                    drive.deactivateDumpServo();
//                    drive.startIntake(1.0);
//                })
//                .splineToConstantHeading(
//                        new Vector2d(33,3), Math.toRadians(0),
//                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajMid3))
//                .build();
//        nextPoseCycleAuto = trajMid2.end();
//
//        trajMid3 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
//                .addTemporalMarker(0.7, () -> {
//                    drive.reverseIntake(1.0);
//                })
//                .lineToConstantHeading(new Vector2d(6, 5))
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajMid4))
//                .build();
//        nextPoseCycleAuto = trajMid3.end();
//
//        trajMid4 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
//                .addTemporalMarker(0, () -> {
//                    drive.activateLift(liftMotorTicks);
//                    drive.activateDumpServoHalf();
//                })
//                .splineTo(new Vector2d(-15,-19), Math.toRadians(-120))
//                .addTemporalMarker(1.2, () -> {
//                    drive.activateDumpServo();
//                })
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajMid5))
//                .build();
//        nextPoseCycleAuto = trajMid4.end();
//
//        trajMid5 = drive.trajectoryBuilder(nextPoseCycleAuto)
//                .splineToSplineHeading(new Pose2d(-5,6, Math.toRadians(0)), Math.toRadians(0))
//                .addTemporalMarker(0, () -> {
//                    drive.deactivateLift();
//                    drive.deactivateDumpServo();
//                    drive.startIntake(1.0);
//                })
//                .splineTo(
//                        new Vector2d(36,8), Math.toRadians(0),
//                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajMid6))
//                .build();
//        nextPoseCycleAuto = trajMid5.end();
//
//        trajMid6 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
//                .addTemporalMarker(0.7, () -> {
//                    drive.reverseIntake(1.0);
//                })
//                .lineToConstantHeading(new Vector2d(6, 10))
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajMid7))
//                .build();
//        nextPoseCycleAuto = trajMid6.end();
//
//        trajMid7 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
//                .addTemporalMarker(0, () -> {
//                    drive.activateLift(liftMotorTicks);
//                    drive.activateDumpServoHalf();
//                })
//                .splineTo(new Vector2d(-13,-12), Math.toRadians(-120))
//                .addTemporalMarker(1.2, () -> {
//                    drive.activateDumpServo();
//                })
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajMid8))
//                .build();
//        nextPoseCycleAuto = trajMid7.end();
//
//        trajMid8 = drive.trajectoryBuilder(nextPoseCycleAuto)
//                .splineToSplineHeading(new Pose2d(-5,10, Math.toRadians(0)), Math.toRadians(0))
//                .addTemporalMarker(0, () -> {
//                    drive.deactivateLift();
//                    drive.deactivateDumpServo();
//                    drive.startIntake(1.0);
//                })
//                .splineTo(
//                        new Vector2d(40,13), Math.toRadians(0),
//                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajMid9))
//                .build();
//        nextPoseCycleAuto = trajMid8.end();
//
//        trajMid9 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
//                .addTemporalMarker(0.7, () -> {
//                    drive.reverseIntake(1.0);
//                })
//                .lineToConstantHeading(new Vector2d(0, 17))
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajMid10))
//                .build();
//        nextPoseCycleAuto = trajMid9.end();
//
//        trajMid10 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
//                .addTemporalMarker(0, () -> {
//                    drive.activateLift(liftMotorTicks);
//                    drive.activateDumpServoHalf();
//                })
//                .splineTo(new Vector2d(-17,-6), Math.toRadians(-120))
//                .addTemporalMarker(1.4, () -> {
//                    drive.activateDumpServo();
//                })
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajMid11))
//                .build();
//        nextPoseCycleAuto = trajMid10.end();
//
//        trajMid11 = drive.trajectoryBuilder(nextPoseCycleAuto)
//                .addTemporalMarker(0, () -> {
//                    drive.deactivateLift();
//                    drive.deactivateDumpServo();
//                    drive.startIntake(1.0);
//                })
//                .splineToSplineHeading(new Pose2d(-5,16, Math.toRadians(0)), Math.toRadians(0))
//                .splineTo(
//                        new Vector2d(40,19), Math.toRadians(0),
//                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajMid12))
//                .build();
//        nextPoseCycleAuto = trajMid11.end();
//
//        trajMid12 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
//                .addTemporalMarker(0.7, () -> {
//                    drive.reverseIntake(1.0);
//                })
//                .lineToConstantHeading(new Vector2d(0, 20))
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajMid13))
//                .build();
//        nextPoseCycleAuto = trajMid12.end();
//
//        trajMid13 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
//                .addTemporalMarker(0, () -> {
//                    drive.activateLift(liftMotorTicks);
//                    drive.activateDumpServoHalf();
//                })
//                .splineTo(new Vector2d(-17,0), Math.toRadians(-120))
//                .addTemporalMarker(1.2, () -> {
//                    drive.activateDumpServo();
//                })
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajMid14))
//                .build();
//        nextPoseCycleAuto = trajMid13.end();
//
//        trajMid14 = drive.trajectoryBuilder(nextPoseCycleAuto)
//                .addTemporalMarker(0, () -> {
//                    drive.deactivateLift();
//                    drive.deactivateDumpServo();
//                    drive.startIntake(1.0);
//                })
//                .splineToSplineHeading(new Pose2d(-5,23, Math.toRadians(0)), Math.toRadians(0))
//                .splineTo(new Vector2d(34,25), Math.toRadians(0))
//                .build();
//        nextPoseCycleAuto = trajMid14.end();
//
//// ******** MID Ends ********
//// ******** Trajectories for HIGH Starts ********
//        trajHigh1 = drive.trajectoryBuilder(startPose, true)
//                .addDisplacementMarker(() -> {
//                    drive.activateDumpServoHalf();
//                    drive.activateLift(liftMotorTicks);
//                })
//                .splineTo(new Vector2d(-14,-24), Math.toRadians(-120))
//                .addTemporalMarker(1.2, () -> {
//                    drive.activateDumpServo();
//                })
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajHigh2))
//                .build();
//        nextPoseCycleAuto = trajHigh1.end();
//
//        trajHigh2 = drive.trajectoryBuilder(nextPoseCycleAuto)
//                .splineToSplineHeading(new Pose2d(-5,2, Math.toRadians(0)), Math.toRadians(0))
//                .addTemporalMarker(0, () -> {
//                    drive.deactivateLift();
//                    drive.deactivateDumpServo();
//                    drive.startIntake(1.0);
//                })
//                .splineToConstantHeading(
//                        new Vector2d(33,3), Math.toRadians(0),
//                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajHigh3))
//                .build();
//        nextPoseCycleAuto = trajHigh2.end();
//
//        trajHigh3 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
//                .addTemporalMarker(0.7, () -> {
//                    drive.reverseIntake(1.0);
//                })
//                .lineToConstantHeading(new Vector2d(6, 5))
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajHigh4))
//                .build();
//        nextPoseCycleAuto = trajHigh3.end();
//
//        trajHigh4 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
//                .addTemporalMarker(0, () -> {
//                    drive.activateLift(liftMotorTicks);
//                    drive.activateDumpServoHalf();
//                })
//                .splineTo(new Vector2d(-15,-19), Math.toRadians(-120))
//                .addTemporalMarker(1.2, () -> {
//                    drive.activateDumpServo();
//                })
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajHigh5))
//                .build();
//        nextPoseCycleAuto = trajHigh4.end();
//
//        trajHigh5 = drive.trajectoryBuilder(nextPoseCycleAuto)
//                .splineToSplineHeading(new Pose2d(-5,6, Math.toRadians(0)), Math.toRadians(0))
//                .addTemporalMarker(0, () -> {
//                    drive.deactivateLift();
//                    drive.deactivateDumpServo();
//                    drive.startIntake(1.0);
//                })
//                .splineTo(
//                        new Vector2d(36,8), Math.toRadians(0),
//                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajHigh6))
//                .build();
//        nextPoseCycleAuto = trajHigh5.end();
//
//        trajHigh6 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
//                .addTemporalMarker(0.7, () -> {
//                    drive.reverseIntake(1.0);
//                })
//                .lineToConstantHeading(new Vector2d(6, 10))
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajHigh7))
//                .build();
//        nextPoseCycleAuto = trajHigh6.end();
//
//        trajHigh7 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
//                .addTemporalMarker(0, () -> {
//                    drive.activateLift(liftMotorTicks);
//                    drive.activateDumpServoHalf();
//                })
//                .splineTo(new Vector2d(-13,-12), Math.toRadians(-120))
//                .addTemporalMarker(1.2, () -> {
//                    drive.activateDumpServo();
//                })
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajHigh8))
//                .build();
//        nextPoseCycleAuto = trajHigh7.end();
//
//        trajHigh8 = drive.trajectoryBuilder(nextPoseCycleAuto)
//                .splineToSplineHeading(new Pose2d(-5,10, Math.toRadians(0)), Math.toRadians(0))
//                .addTemporalMarker(0, () -> {
//                    drive.deactivateLift();
//                    drive.deactivateDumpServo();
//                    drive.startIntake(1.0);
//                })
//                .splineTo(
//                        new Vector2d(40,13), Math.toRadians(0),
//                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajHigh9))
//                .build();
//        nextPoseCycleAuto = trajHigh8.end();
//
//        trajHigh9 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
//                .addTemporalMarker(0.7, () -> {
//                    drive.reverseIntake(1.0);
//                })
//                .lineToConstantHeading(new Vector2d(0, 17))
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajHigh10))
//                .build();
//        nextPoseCycleAuto = trajHigh9.end();
//
//        trajHigh10 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
//                .addTemporalMarker(0, () -> {
//                    drive.activateLift(liftMotorTicks);
//                    drive.activateDumpServoHalf();
//                })
//                .splineTo(new Vector2d(-17,-6), Math.toRadians(-120))
//                .addTemporalMarker(1.4, () -> {
//                    drive.activateDumpServo();
//                })
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajHigh11))
//                .build();
//        nextPoseCycleAuto = trajHigh10.end();
//
//        trajHigh11 = drive.trajectoryBuilder(nextPoseCycleAuto)
//                .addTemporalMarker(0, () -> {
//                    drive.deactivateLift();
//                    drive.deactivateDumpServo();
//                    drive.startIntake(1.0);
//                })
//                .splineToSplineHeading(new Pose2d(-5,16, Math.toRadians(0)), Math.toRadians(0))
//                .splineTo(
//                        new Vector2d(40,19), Math.toRadians(0),
//                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajHigh12))
//                .build();
//        nextPoseCycleAuto = trajHigh11.end();
//
//        trajHigh12 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
//                .addTemporalMarker(0.7, () -> {
//                    drive.reverseIntake(1.0);
//                })
//                .lineToConstantHeading(new Vector2d(0, 20))
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajHigh13))
//                .build();
//        nextPoseCycleAuto = trajHigh12.end();
//
//        trajHigh13 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
//                .addTemporalMarker(0, () -> {
//                    drive.activateLift(liftMotorTicks);
//                    drive.activateDumpServoHalf();
//                })
//                .splineTo(new Vector2d(-17,0), Math.toRadians(-120))
//                .addTemporalMarker(1.2, () -> {
//                    drive.activateDumpServo();
//                })
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajHigh14))
//                .build();
//        nextPoseCycleAuto = trajHigh13.end();
//
//        trajHigh14 = drive.trajectoryBuilder(nextPoseCycleAuto)
//                .addTemporalMarker(0, () -> {
//                    drive.deactivateLift();
//                    drive.deactivateDumpServo();
//                    drive.startIntake(1.0);
//                })
//                .splineToSplineHeading(new Pose2d(-5,23, Math.toRadians(0)), Math.toRadians(0))
//                .splineTo(new Vector2d(34,25), Math.toRadians(0))
//                .build();
//        nextPoseCycleAuto = trajHigh14.end();
//// ******** HIGH Ends ********
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        elementPosition = BarcodeDetectionBluePipeline.position;
//        telemetry.addData("Position: ", elementPosition);
//        telemetry.update();
//
//        if(elementPosition == 1) {
//            drive.followTrajectoryAsync(trajLow1);
//            //drive.activateLift(TICKS_FOR_ONE);
//        } else if(elementPosition == 2) {
//            drive.followTrajectoryAsync(trajMid1);
//            //drive.activateLift(TICKS_FOR_TWO);
//        } else {
//            //position = 3
//            drive.followTrajectoryAsync(trajHigh1);
//            //drive.activateLift(TICKS_FOR_THREE);
//        }
//
//        while (opModeIsActive() && !isStopRequested()) {
//            drive.update();
//        }
    }

//    private void initialize() {
//        drive = new RRAutoDriveBlue(hardwareMap);
//        drive.startCamera();
//        BarcodeDetectionBluePipeline.setRegionValues(6, 72, 5);
//        telemetry.addData("Status:", "initialize() - Robot and Camera are initialized!");
//        telemetry.update();
//    }

}