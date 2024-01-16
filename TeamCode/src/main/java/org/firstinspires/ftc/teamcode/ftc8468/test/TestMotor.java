package org.firstinspires.ftc.teamcode.ftc8468.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class TestMotor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Change deviceName to test other servos
        DcMotorEx  vertMotorL = hardwareMap.get(DcMotorEx.class, "leftVert");
        DcMotorEx  vertMotorR = hardwareMap.get(DcMotorEx.class, "rightVert");
        vertMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
//        DcMotorEx  horizMotorL = hardwareMap.get(DcMotorEx.class, "leftHoriz");
//        DcMotorEx  horizMotorR = hardwareMap.get(DcMotorEx.class, "rightHoriz");

        ElapsedTime elapsedTime = new ElapsedTime();

        waitForStart();

        elapsedTime.reset();
        double power = 1.0;
        while (opModeIsActive()) {
            if(gamepad1.a) {
                vertMotorL.setPower(power);
                vertMotorR.setPower(power);
            }
            if(gamepad1.b) {
                vertMotorL.setPower(-power);
                vertMotorR.setPower(-power);
            }
//            if(gamepad1.x) {
//                horizMotorL.setPower(power);
//                horizMotorR.setPower(power);
//            }
//
//            if(gamepad1.y) {
//                horizMotorL.setPower(-power);
//                horizMotorR.setPower(-power);
//            }

            idle();
        }
    }
}
