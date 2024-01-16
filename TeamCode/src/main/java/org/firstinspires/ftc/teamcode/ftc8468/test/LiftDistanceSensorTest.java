package org.firstinspires.ftc.teamcode.ftc8468.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ftc8468.test.camera.RobotDriveTemp;


@Autonomous
public class LiftDistanceSensorTest extends LinearOpMode {
    RobotDriveTemp drive;
    private static int elementPosition = 0;

    protected Servo turretServo;

    public void runOpMode() {
        initialize();

        waitForStart();
        if (isStopRequested()) return;

        int[] returnValues = new int[] {0, 0};
        while (opModeIsActive() && !isStopRequested()) {
//            if(pipeline.isBlueVisible()) {
//                double imageCenterX = pipeline.getCenterofRect(pipeline.getBlueRect()).x;
//                telemetry.addData("Data: Center - ", centerX + " Image Center: " + imageCenterX);
//            } else {
//                telemetry.addData("Data: ", "There is no Blue Objects in the frame!");
//            }
//            telemetry.update();

//            if(!isAligned) {
//                isAligned = drive.alignTurret(telemetry);
//            }
            double distance = drive.getDistance();
            if(distance > 10.0) {
                returnValues = drive.checkAndMoveLift(300, 20);
            } else {
                drive.stopLift();;
            }
            telemetry.addData("distance: ", distance);
            telemetry.update();
        }
    }

    private void initialize() {
        drive = new RobotDriveTemp(hardwareMap);
//        telemetry.addData("Status:", "initialize() - Robot and Camera are initialized!");
//        telemetry.update();
    }

}
