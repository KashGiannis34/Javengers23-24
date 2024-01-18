package org.firstinspires.ftc.teamcode.ftc8468.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class TestSensor extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Change deviceName to test other servos
        DigitalChannel sensor1 = hardwareMap.get(DigitalChannel.class, "touchSensor1");
        sensor1.setMode(DigitalChannel.Mode.INPUT);
        ElapsedTime elapsedTime = new ElapsedTime();

        waitForStart();

        elapsedTime.reset();
        while (opModeIsActive()) {
            if (sensor1.getState() == true) {
                telemetry.addData("Digital Touch", "Is Not Pressed");
            } else {
                telemetry.addData("Digital Touch", "Is Pressed");
            }
            telemetry.update();
        }
    }

}
