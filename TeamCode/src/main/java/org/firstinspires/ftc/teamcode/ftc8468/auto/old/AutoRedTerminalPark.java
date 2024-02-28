package org.firstinspires.ftc.teamcode.ftc8468.auto.old;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ftc8468.auto.RRAutoDrive;

@Autonomous
@Disabled
public class AutoRedTerminalPark extends LinearOpMode {
    RRAutoDrive drive;
    private static int position = 0;

    // RPM = 1620; TICKS_PER_REV = 103.8; Gear Ratio = 5;
    int TICKS_FOR_ONE = 519; // Ticks for 1 rotation is 103.8 * 5 = 58;
    int TICKS_FOR_TWO = 1038; // Ticks for 2 rotation is 28 * 10 = 1538;
    int TICKS_FOR_THREE = 1557; // Ticks for 3 rotation is 28 * 15 = 2307;
    private int liftMotorTicks = TICKS_FOR_THREE;


    public void runOpMode() {
        initialize();

        Pose2d startPose = new Pose2d();
        Pose2d nextPose = startPose;
        Pose2d nextPoseCycleAuto;

        drive.setPoseEstimate(startPose);

        //FIRST PARK
//        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
//                .addDisplacementMarker(() -> {
//                    drive.deactivateHorizSlide();
//                    drive.activateVertFourbarPrep();
//                    drive.prepVertClaw();
//                    drive.activateMainTurretRed();
//                    drive.activateHorizFourbarHalf();
//                })
//                .lineToLinearHeading(new Pose2d(-55,3, Math.toRadians(-90)),
//                        SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .UNSTABLE_addTemporalMarkerOffset(3, () -> {
//                    drive.deactivateFrontClaw();
//                    drive.activateVertClaw();
//                    drive.activateVertFourbar();
//                    drive.activateVertSlide(725);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(3, () -> {
//                    drive.deactivateVertClaw();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    drive.deactivateVertClaw();
//                })
//                .lineToLinearHeading(new Pose2d(-45,-20, Math.toRadians(-90)),
//                        SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
//
//
//        //SECOND PARK
//        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(startPose)
//                .addDisplacementMarker(() -> {
//                    drive.deactivateHorizSlide();
//                    drive.activateVertFourbarPrep();
//                    drive.prepVertClaw();
//                    drive.activateMainTurretRed();
//                    drive.activateHorizFourbarHalf();
//                })
//                .lineToLinearHeading(new Pose2d(-55,3, Math.toRadians(-90)),
//                        SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .UNSTABLE_addTemporalMarkerOffset(3, () -> {
//                    drive.deactivateFrontClaw();
//                    drive.activateVertClaw();
//                    drive.activateVertFourbar();
//                    drive.activateVertSlide(725);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(3, () -> {
//                    drive.deactivateVertClaw();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    drive.deactivateVertClaw();
//                })
//                .lineToLinearHeading(new Pose2d(-45,0, Math.toRadians(-90)),
//                        SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//
//                .build();
//
//        //THIRD PARK
//        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(startPose)
//                .addDisplacementMarker(() -> {
//                    drive.deactivateHorizSlide();
//                    drive.activateVertFourbarPrep();
//                    drive.prepVertClaw();
//                    drive.activateMainTurretRed();
//                    drive.activateHorizFourbarHalf();
//                })
//                .lineToLinearHeading(new Pose2d(-55,3, Math.toRadians(-90)),
//                        SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .UNSTABLE_addTemporalMarkerOffset(3, () -> {
//                    drive.deactivateFrontClaw();
//                    drive.activateVertClaw();
//                    drive.activateVertFourbar();
//                    drive.activateVertSlide(725);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(3, () -> {
//                    drive.deactivateVertClaw();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    drive.deactivateVertClaw();
//                })
//                .lineToLinearHeading(new Pose2d(-45,20, Math.toRadians(-90)),
//                        SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//
//                .build();

        waitForStart();
        if (isStopRequested()) return;

//         position = drive.getImagePosition();
////        position = 1;
//        telemetry.addData("Position: ", position);
//        telemetry.update();
//
//        if(position == 1) {
//            drive.followTrajectorySequenceAsync(trajSeq1);
//        } else if(position == 2) {
//            drive.followTrajectorySequenceAsync(trajSeq2);
//        } else {
//            //position = 3
//            drive.followTrajectorySequenceAsync(trajSeq3);
//        }

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
