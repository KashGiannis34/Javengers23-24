package org.firstinspires.ftc.teamcode.ftc18305.auto.old;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ftc18305.auto.RRAutoDrive;

@Disabled
@Autonomous
public class Auto_Blue_LOW extends LinearOpMode {
    RRAutoDrive drive;

    private int armTicksQuarter = 200;

    // RPM = 435; TICKS_PER_REV = 384.5; Gear Ratio = 2;
    int TICKS_FOR_ONE = 769; // Ticks for 1 rotation is 384.5 * 2 = 769;
    int TICKS_FOR_TWO = 1538; // Ticks for 2 rotation is 384.5 * 4 = 1538;
    int TICKS_FOR_THREE = 2307; // Ticks for 3 rotation is 384.5 * 6 = 2307;
    int TICKS_FOR_FOURADJUSTED = 3100; // Ticks for 4 rotation is 384.5 * 8 = 3076;
    int TICKS_FOR_FIVE = 3845; // Ticks for 5 rotation is 384.5 * 10 = 3845;
    private int liftMotorTicks = TICKS_FOR_FOURADJUSTED;

    Trajectory traj1, traj2, traj3, traj4, traj5, traj6, traj7, traj8, traj9, traj10, traj11, traj12, traj13;



    public void runOpMode() {
        initialize();

        Pose2d startPose = new Pose2d();
        Pose2d nextPose = startPose;
        Pose2d nextPoseCycleAuto;

        drive.setPoseEstimate(startPose);

        nextPoseCycleAuto = startPose;

        /*
        traj1 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
                .addDisplacementMarker(() -> {
                    drive.activateDumpServoHalf();
                })
                .splineTo(new Vector2d(-14,-24), Math.toRadians(-120))
                .addTemporalMarker(1.2, () -> {
                    drive.activateDumpServo();
                })
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(traj2))
                .build();
        nextPoseCycleAuto = traj1.end();

        traj2 = drive.trajectoryBuilder(nextPoseCycleAuto)
                .splineToSplineHeading(new Pose2d(-5,2, Math.toRadians(0)), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    drive.deactivateDumpServo();
                    drive.startIntake(1.0);
                })
                .splineToConstantHeading(
                        new Vector2d(33,3), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(traj3))
                .build();
        nextPoseCycleAuto = traj2.end();

        traj3 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
                .addTemporalMarker(0.7, () -> {
                    drive.reverseIntake(1.0);
                })
                .lineToConstantHeading(new Vector2d(6, 5))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(traj4))
                .build();
        nextPoseCycleAuto = traj3.end();

        traj4 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
                .addTemporalMarker(0, () -> {
                    drive.activateLift(liftMotorTicks);
                    drive.activateDumpServoHalf();
                })
                .splineTo(new Vector2d(-15,-19), Math.toRadians(-120))
                .addTemporalMarker(1.2, () -> {
                    drive.activateDumpServo();
                })
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(traj5))
                .build();
        nextPoseCycleAuto = traj4.end();

        traj5 = drive.trajectoryBuilder(nextPoseCycleAuto)
                .splineToSplineHeading(new Pose2d(-5,6, Math.toRadians(0)), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    drive.deactivateLift();
                    drive.deactivateDumpServo();
                    drive.startIntake(1.0);
                })
                .splineTo(
                        new Vector2d(36,8), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(traj6))
                .build();
        nextPoseCycleAuto = traj5.end();

        traj6 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
                .addTemporalMarker(0.7, () -> {
                    drive.reverseIntake(1.0);
                })
                .lineToConstantHeading(new Vector2d(6, 10))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(traj7))
                .build();
        nextPoseCycleAuto = traj6.end();

        traj7 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
                .addTemporalMarker(0, () -> {
                    drive.activateLift(liftMotorTicks);
                    drive.activateDumpServoHalf();
                })
                .splineTo(new Vector2d(-13,-12), Math.toRadians(-120))
                .addTemporalMarker(1.2, () -> {
                    drive.activateDumpServo();
                })
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(traj8))
                .build();
        nextPoseCycleAuto = traj7.end();

        traj8 = drive.trajectoryBuilder(nextPoseCycleAuto)
                .splineToSplineHeading(new Pose2d(-5,10, Math.toRadians(0)), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    drive.deactivateLift();
                    drive.deactivateDumpServo();
                    drive.startIntake(1.0);
                })
                .splineTo(
                        new Vector2d(40,13), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(traj9))
                .build();
        nextPoseCycleAuto = traj8.end();

        traj9 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
                .addTemporalMarker(0.7, () -> {
                    drive.reverseIntake(1.0);
                })
                .lineToConstantHeading(new Vector2d(0, 17))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(traj10))
                .build();
        nextPoseCycleAuto = traj9.end();

        traj10 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
                .addTemporalMarker(0, () -> {
                    drive.activateLift(liftMotorTicks);
                    drive.activateDumpServoHalf();
                })
                .splineTo(new Vector2d(-17,-6), Math.toRadians(-120))
                .addTemporalMarker(1.4, () -> {
                    drive.activateDumpServo();
                })
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(traj11))
                .build();
        nextPoseCycleAuto = traj10.end();

        traj11 = drive.trajectoryBuilder(nextPoseCycleAuto)
                .addTemporalMarker(0, () -> {
                    drive.deactivateLift();
                    drive.deactivateDumpServo();
                    drive.startIntake(1.0);
                })
                .splineToSplineHeading(new Pose2d(-5,16, Math.toRadians(0)), Math.toRadians(0))
                .splineTo(new Vector2d(27,18), Math.toRadians(0))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(traj12))
                .build();
        nextPoseCycleAuto = traj11.end();

        traj12 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
                .lineTo(new Vector2d(29,-10))
                .build();
        nextPoseCycleAuto = traj12.end();
//
//        traj13 = drive.trajectoryBuilder(nextPoseCycleAuto, true)
//                .splineTo(new Vector2d(-22,-5), Math.toRadians(-90))
//                .build();
//        nextPoseCycleAuto = traj13.end();
        // *** Trajectories for 0 Ring Auto Ends ***
*/
        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectoryAsync(traj1);

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();

        }

    }

    private void initialize() {
        drive = new RRAutoDrive(hardwareMap);

    }

}
