package org.firstinspires.ftc.teamcode.ftc8468.test.camera.sample;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ftc8468.test.camera.RobotDriveYellow;


@Autonomous
public class VisionYellowTest extends LinearOpMode {
    RobotDriveYellow drive;
    YellowPolePipelineOld pipeline;
    private static int elementPosition = 0;

    public void runOpMode() {
        initialize();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status:", "running!");
            telemetry.update();
        }
    }

    private void initialize() {
        drive = new RobotDriveYellow(hardwareMap);
        pipeline = new YellowPolePipelineOld();
        drive.startCamera(pipeline);
        telemetry.addData("Status:", "initialize() - Robot and Camera are initialized!");
        telemetry.update();
    }

}
