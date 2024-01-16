package org.firstinspires.ftc.teamcode.ftc18305.test.camera.sample;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ftc18305.test.camera.RobotDriveTemp;


@Autonomous
public class VisionBlueRedTest extends LinearOpMode {
    RobotDriveTemp drive;
    BlueConePipeline pipeline;
    private static int elementPosition = 0;

    protected Servo turretServo;

    public void runOpMode() {
        initialize();

        waitForStart();
        if (isStopRequested()) return;

        double centerX = pipeline.centerX;
        boolean isAligned = false;
        while (opModeIsActive() && !isStopRequested()) {
//            if(pipeline.isBlueVisible()) {
//                double imageCenterX = pipeline.getCenterofRect(pipeline.getBlueRect()).x;
//                telemetry.addData("Data: Center - ", centerX + " Image Center: " + imageCenterX);
//            } else {
//                telemetry.addData("Data: ", "There is no Blue Objects in the frame!");
//            }
//            telemetry.update();

//            if(!isAligned) {
                isAligned = drive.alignTurret(telemetry);
//            }
            telemetry.addData("isAligned: ", isAligned);
            telemetry.update();
        }
    }

    private void initialize() {
        drive = new RobotDriveTemp(hardwareMap);
        pipeline = new BlueConePipeline();
        drive.startCamera();
        telemetry.addData("Status:", "initialize() - Robot and Camera are initialized!");
        telemetry.update();
    }

}
