package org.firstinspires.ftc.teamcode.ftc8468.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ftc8468.auto.pipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.ftc8468.auto.temp.VisionAutoDrive;


@Autonomous
public class AlignTurretTest extends LinearOpMode {
    VisionAutoDrive drive;
    private static int position = 0;
    boolean isAligned = false;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    protected Servo turretServo;

    public void runOpMode() {
        initialize();

        waitForStart();
        if (isStopRequested()) return;

        position = drive.getImagePosition();
        telemetry.addData("Position: ", position);
        telemetry.update();

        if(position == 1) {
        } else if(position == 2) {
        } else {
        }

        drive.switchCameraToPolePipeline();

        int[] returnValues = new int[] {0, 0};
        while (opModeIsActive() && !isStopRequested()) {
            // wait for 5 seconds...
            sleep(5000);

            if(!isAligned) {
                isAligned = drive.alignTurret(telemetry);
            }
        }
    }

    private void initialize() {
        drive = new VisionAutoDrive(hardwareMap);
        AprilTagDetectionPipeline pipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        drive.startCamera(pipeline);
        telemetry.addData("Status:", "initialize() - Robot and Camera are initialized!");
        telemetry.update();
    }

}
