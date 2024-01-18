package org.firstinspires.ftc.teamcode.ftc8468.test.camera.sample;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ftc8468.test.camera.RobotDriveTemp;


@Autonomous
public class VisionTest extends LinearOpMode {
    RobotDriveTemp drive;
    UGAngleHighGoalPipeline pipeline;
    private static int elementPosition = 0;

    public void runOpMode() {
        initialize();

        waitForStart();
        if (isStopRequested()) return;

        telemetry.addData("status: ", "pipeline data is updated");
        telemetry.update();

        while (opModeIsActive() && !isStopRequested()) {

            double angle = pipeline.calculateYaw(UGAngleHighGoalPipeline.Target.BLUE);
            double pitch = pipeline.calculatePitch(UGAngleHighGoalPipeline.Target.BLUE);
            telemetry.addData("Yaw | Pitch: ", angle + " | ", pitch);
            telemetry.update();
        }

    }

    private void initialize() {
        drive = new RobotDriveTemp(hardwareMap);
        pipeline = new UGAngleHighGoalPipeline(0.0, 0.0, 0.0);
        drive.startCamera(pipeline);
        telemetry.addData("Status:", "initialize() - Robot and Camera are initialized!");
        telemetry.update();
    }

}
