package org.firstinspires.ftc.teamcode.ftc8468.auto.old;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ftc8468.auto.RRAutoDrive;
import org.firstinspires.ftc.teamcode.ftc8468.auto.pipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous
@Disabled
public class AutoBlueTerminalPark extends LinearOpMode {
    RRAutoDrive drive;
    private static int position = 0;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    //int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    int FIRST = 5;
    int SECOND = 10;
    int THIRD = 11;

    Trajectory traj1;

    public static final int CAMERA_WIDTH = 800; //1280;
    public static final int CAMERA_HEIGHT = 448; //720;

    protected float speedMultiplier = 0.6f;

    final int LIFT_TOLERANCE = 10;
    final int VERT_TOLERANCE = 0;
    final int HORIZ_TOLERANCE = 0;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    static final double FEET_PER_METER = 3.28084;
    int numFramesWithoutDetection = 0;

    TrajectorySequence trajSeqLow, trajSeqMid, trajSeqHigh;

    // RPM = 1620; TICKS_PER_REV = 103.8; Gear Ratio = 5;
    int TICKS_FOR_HIGH = 725;
    private int vertLiftTicks = TICKS_FOR_HIGH;


    public void runOpMode() {
        initialize();

        Pose2d startPose = new Pose2d();
        Pose2d nextPose = startPose;
        Pose2d nextPoseCycleAuto;

        drive.setPoseEstimate(startPose);
// Low Starts Here...
        nextPoseCycleAuto = startPose;
        //FIRST PARK
//        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
//                .addDisplacementMarker(() -> {
//                    drive.deactivateHorizSlide();
//                    drive.activateVertFourbarPrep();
//                    drive.prepVertClaw();
//                    drive.activateMainTurretRed();
//                    drive.activateHorizFourbarHalf();
//                })
//                .lineToLinearHeading(new Pose2d(-55,-3, Math.toRadians(90)),
//                        SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .UNSTABLE_addTemporalMarkerOffset(3, () -> {
//                    drive.deactivateFrontClaw();
//                    drive.activateVertClaw();
//                    drive.activateVertFourbar();
//                    drive.activateVertSlide(vertLiftTicks);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(3, () -> {
//                    drive.deactivateVertClaw();
//                    drive.restVertFourbar();
//                })
//                .lineToLinearHeading(new Pose2d(-45,20, Math.toRadians(90)),
//                        SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .UNSTABLE_addDisplacementMarkerOffset(0,() ->{
//                    drive.deactivateVertSlide();
//                })
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
//                .lineToLinearHeading(new Pose2d(-55,-3, Math.toRadians(90)),
//                        SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .UNSTABLE_addTemporalMarkerOffset(3, () -> {
//                    drive.deactivateFrontClaw();
//                    drive.activateVertClaw();
//                    drive.activateVertFourbar();
//                    drive.activateVertSlide(vertLiftTicks);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(3, () -> {
//                    drive.deactivateVertClaw();
//                    drive.restVertFourbar();
//                })
//
//                .lineToLinearHeading(new Pose2d(-45,0, Math.toRadians(90)),
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
//                .lineToLinearHeading(new Pose2d(-55,-3, Math.toRadians(90)),
//                        SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .UNSTABLE_addTemporalMarkerOffset(3, () -> {
//                    drive.deactivateFrontClaw();
//                    drive.activateVertClaw();
//                    drive.activateVertFourbar();
//                    drive.activateVertSlide(vertLiftTicks);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(3, () -> {
//                    drive.deactivateVertClaw();
//                    drive.restVertFourbar();
//                })
//                .lineToLinearHeading(new Pose2d(-45,20, Math.toRadians(90)),
//                        SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//
//                .build();

        waitForStart();
        if (isStopRequested()) return;

//        position = drive.getImagePosition();
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
            telemetry.addData("Position (loop): ", position);
            telemetry.update();
        }

    }

    private void initialize() {
        drive = new RRAutoDrive(hardwareMap);
//        drive.startCamera();
        telemetry.addData("Status:", "initialize() - Robot and Camera are initialized!");
        telemetry.update();
    }

//    private void startCamera() {
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "WebcamR"), cameraMonitorViewId);
//        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
//
//        camera.setPipeline(aprilTagDetectionPipeline);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                camera.startStreaming(800,448, OpenCvCameraRotation.UPSIDE_DOWN);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//
//            }
//        });
//    }
//
//    private int getImagePosition() {
//        int position = 0;
//        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
//
//        // If there's been a new frame...
//        if(detections != null) {
//            telemetry.addData("FPS", camera.getFps());
//            telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
//            telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());
//
//            // If we don't see any tags
//            if(detections.size() == 0) {
//                numFramesWithoutDetection++;
//
//                // If we haven't seen a tag for a few frames, lower the decimation
//                // so we can hopefully pick one up if we're e.g. far back
//                if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
//                    aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
//                }
//            }
//            // We do see tags!
//            else {
//                numFramesWithoutDetection = 0;
//
//                // If the target is within 1 meter, turn on high decimation to
//                // increase the frame rate
//                if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
//                    aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
//                }
//
//                for(AprilTagDetection detection : detections) {
//                    telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//                    telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
//                    telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
//                    telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//                    telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//                    telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//                    telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
//
//                    if (detection.id == FIRST || detection.id == SECOND || detection.id == THIRD) {
//                        position = detection.id;
//                        if (detection.id == FIRST) {
//                            position = 1;
//                        } else if (detection.id == SECOND) {
//                            position = 2;
//                        } else if (detection.id == THIRD) {
//                            position = 3;
//                        }
//                        break;
//                    }
//                }
//            }
//            telemetry.update();
//        }
//        return position;
//    }

}
