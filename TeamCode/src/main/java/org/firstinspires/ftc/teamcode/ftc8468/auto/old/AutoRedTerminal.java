package org.firstinspires.ftc.teamcode.ftc8468.auto.old;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ftc8468.auto.RRAutoDrive;

@Autonomous
public class AutoRedTerminal extends LinearOpMode {
    RRAutoDrive drive;
    private static int position = 0;

    int TICKS_FOR_HIGH = 760;
    private int liftVertTicks = TICKS_FOR_HIGH;

    int TICKS_FOR_HORIZ = 580;
    private int horizLiftTicks = TICKS_FOR_HORIZ;

    public void runOpMode() {
        initialize();

        Pose2d startPose = new Pose2d();

        drive.setPoseEstimate(startPose);

//        //PARK ZONE 1 CYCLE AUTO
//        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
//                .addDisplacementMarker(() -> {
//                    drive.deactivateHorizSlide();
//                    drive.activateVertFourbarGrab();
//                    drive.deactivateVertClaw();
//                    drive.activateMainTurretRed();
//                    drive.activateHorizFourbarHalf();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    drive.activateVertClaw();
//                })
//                .lineToLinearHeading(new Pose2d(-57,-2, Math.toRadians(-90)),
//                        SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
////                    drive.disengageHorizLock();
////                    drive.activateHorizSlide(horizLiftTicks);
////                    drive.activateHorizFourbar();
//                    drive.deactivateFrontClaw();
//                    drive.activateVertClaw();
////                    drive.activateVertFourbar();
//                    drive.activateVertSlide(liftVertTicks);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
//                    drive.activateVertFourbar();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                    drive.deactivateVertClaw();
//                    drive.restVertFourbar()
//                    ;
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
//                    drive.deactivateVertSlide();
//                    drive.restMainTurret();
//                })
//                .lineToLinearHeading(new Pose2d(-57,-3, Math.toRadians(-90)),
//                        SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.    MAX_ACCEL))
//                .waitSeconds(3)
//                .lineToLinearHeading(new Pose2d(-40,-5, Math.toRadians(0)))

//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    drive.deactivateVertClaw();
//                    drive.deactivateHorizFourbar();
//                    drive.activateFrontClaw();
//
//                })
//
//
//                .build();

// Low Starts Here...
//        nextPoseCycleAuto = startPose;
//        traj1 = drive.trajectoryBuilder(startPose, true)
//                .addDisplacementMarker(() -> {
//                    drive.deactivateHorizSlide();
//                    drive.activateVertFourbarPrep();
//                    drive.prepVertClaw();
//                    drive.activateMainTurretRed();
//                })
//                .lineToLinearHeading(new Pose2d(-55,3, Math.toRadians(-90)),
//                        SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
////                .splineTo(new Vector2d(-55,3), Math.toRadians(90))
//                .addDisplacementMarker(() -> {
//                    drive.activateHorizSlide(580);
//                    drive.activateHorizFourbar();
//                    drive.deactivateFrontClaw();
//                    drive.activateVertClaw();
//                    drive.activateVertFourbar();
//                    drive.activateVertSlide(725);
//                })
//
////                .addDisplacementMarker(() -> drive.followTrajectoryAsync(traj2))
//                .build();
//        nextPoseCycleAuto = traj1.end();
//
//        traj2 = drive.trajectoryBuilder(nextPoseCycleAuto)
//                .lineToConstantHeading(new Vector2d(-55,3.1))
//                .addDisplacementMarker(() -> {
//                    drive.activateMainTurretRight();
//                })
//                .build();
//        nextPoseCycleAuto = traj2.end();


        waitForStart();
        if (isStopRequested()) return;

//         position = drive.getImagePosition();
//        position = 1;
//         telemetry.addData("Position: ", position);
//         telemetry.update();
//
//         if(position == 1) {
//            drive.followTrajectorySequenceAsync(trajSeq1);
//         } else if(position == 2) {
//         //drive.followTrajectorySequenceAsync(trajSeqMid);
//         } else {
//         //position = 3
//         //drive.followTrajectorySequenceAsync(trajSeqHigh);
//         }

         // Switch the pipeline, so that Blue/Red cones can be detected and the intake claw can position itself appropriately...
//        drive.switchCameraToConePipeline();
//         boolean isAligned = false;

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();

//            // need to align the Intake Turret only while extending the Horizontal Slide..
//            isAligned = drive.alignIntakeTurret(telemetry);
//
//            drive.checkLiftMotion();

//            position = drive.getImagePosition();
//            telemetry.addData("Position (loop): ", position);
//            telemetry.update();
        }

    }

    private void initialize() {
        drive = new RRAutoDrive(hardwareMap);
//        drive.startCamera();
        telemetry.addData("Status:", "initialize() - Robot and Camera are initialized!");
        telemetry.update();
    }

}
