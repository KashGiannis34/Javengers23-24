package org.firstinspires.ftc.teamcode.ftc8468.test;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Sensor: ColorRev", group = "Sensor")
public class RevColorSensorTest extends LinearOpMode {

    RevColorSensorV3 colorSensor, colorSensor2;

    @Override public void runOpMode() {
        final float[] hsvValues = new float[3];
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "leftColorSensor");
        colorSensor2 = hardwareMap.get(RevColorSensorV3.class, "rightColorSensor");
        colorSensor.setGain(2.0f);

        waitForStart();
        if(isStopRequested()) return;

        while (opModeIsActive()) {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);


            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsvValues[0])
                    .addData("Saturation", "%.3f", hsvValues[1])
                    .addData("Value", "%.3f", hsvValues[2]);
            telemetry.addData("Alpha", "%.3f", colors.alpha);

            telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
            telemetry.addData("Distance2 (cm)", "%.3f", ((DistanceSensor) colorSensor2).getDistance(DistanceUnit.CM));

            telemetry.update();
        }

    }

}
