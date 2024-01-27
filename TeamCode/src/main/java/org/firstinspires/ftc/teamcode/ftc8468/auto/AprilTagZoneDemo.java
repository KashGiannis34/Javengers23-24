/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.ftc8468.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

//@TeleOp
@Autonomous
public class AprilTagZoneDemo extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

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

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    int TARGET_ID = 4;
    boolean gamepadXisPressed = false;
    boolean gamepadBisPressed = false;
    int action = -1;
    int index = 0;

    double headingPower = 0;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "WebcamR"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        RRAutoDrive drive = new RRAutoDrive(hardwareMap);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        waitForStart();

        telemetry.setMsTransmissionInterval(50);
        double heading;
        double rawHeading;
        while (opModeIsActive())
        {
            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
            rawHeading = drive.getPoseEstimate().getHeading();
            if (rawHeading >= Math.PI)
                heading = rawHeading-2*Math.PI;
            else
                heading = rawHeading;

            if (Math.abs(heading) > 0.04) {
                if (heading < 0)
                    headingPower = Range.clip(Math.abs(6 * (heading / (2 * Math.PI))) + Math.abs(9*Math.pow((heading / (2 * Math.PI)),2)), 0.15, 0.5);
                else
                    headingPower = -1 * Range.clip(Math.abs(6 * (heading / (2 * Math.PI))) + Math.abs(9*Math.pow((heading / (2 * Math.PI)), 2)), 0.15, 0.5);
            }
            else
                headingPower = 0;

            if (gamepad1.x)
                gamepadXisPressed = true;
            if (gamepadXisPressed && !gamepad1.x)
            {
                TARGET_ID--;
                action = -2;
                gamepadXisPressed = false;
            }

            if (gamepad1.b)
                gamepadBisPressed = true;
            if (gamepadBisPressed && !gamepad1.b)
            {
                TARGET_ID++;
                action = -2;
                gamepadBisPressed = false;
            }


            // If there's been a new frame...
            if(detections != null)
            {
                telemetry.addData("FPS", camera.getFps());
                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());
                telemetry.addData("Robot pose: ", drive.getPoseEstimate());
                telemetry.addData("Action: ", action);
                telemetry.addData("Target ID: ", TARGET_ID);
                telemetry.addData("Heading Power: ", headingPower);
                telemetry.addData("Heading: ", heading);
                telemetry.addData("Raw Heading: ", rawHeading);

                // If we don't see any tags
                if(detections.size() == 0)
                {
                    drive.setWeightedDrivePower(new Pose2d(0,0,headingPower));
                }
                // We do see tags!
                else
                {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }

                    int curIndex = -1;
                    for(AprilTagDetection detection : detections)
                    {
                        curIndex++;
                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//                        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//                        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//                        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
                        if (detection.id == TARGET_ID) {
                            action = 0;
                            index = curIndex;
                        }
                        if (detection.id < TARGET_ID && action != 0)
                            action = 1;
                        if (detection.id > TARGET_ID && action != 0)
                            action = -1;
                    }

                    if (action == 0) {
                        if (index < detections.size()) {
                            if (detections.get(index).pose.x > .1)
                                drive.setWeightedDrivePower(new Pose2d(0, 0.4, headingPower));
                            else if (detections.get(index).pose.x < -.1)
                                drive.setWeightedDrivePower(new Pose2d(0, -0.4, headingPower));
                            else
                                drive.setWeightedDrivePower(new Pose2d(0, 0, headingPower));
                        }
                        else
                            drive.setWeightedDrivePower(new Pose2d(0, 0, headingPower));
                    }
                    else if (action == -1)
                    {
                        drive.setWeightedDrivePower(new Pose2d(0,-0.4, headingPower));
                    }
                    else
                    {
                        drive.setWeightedDrivePower(new Pose2d(0,0.4, headingPower));
                    }




                }
                drive.update();
            }
            telemetry.update();
        }
    }
}
