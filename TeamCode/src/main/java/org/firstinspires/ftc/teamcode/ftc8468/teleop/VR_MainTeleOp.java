package org.firstinspires.ftc.teamcode.ftc8468.teleop;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ftc8468.RobotConstants;


@TeleOp(name = "VR_MainTeleOp", group = "ftc8468")
public class VR_MainTeleOp extends LinearOpMode {
    private double wheelMotorSpeed = 0.7;
    private int shooterTicks = 2475;
    private int shooterTicksEndGame = 2375;

    private boolean overrideAutoClaw = true;
    private boolean overrideButtonPressed = false;
    boolean bPressed = false;
    boolean climbServoActivated = false;

    /**
     // RPM = 435; TICKS_PER_REV = 384.5; Gear Ratio = 2;
     int TICKS_FOR_ONE = 769; // Ticks for 1 rotation is 384.5 * 2 = 769;
     int TICKS_FOR_TWO = 1538; // Ticks for 2 rotation is 384.5 * 4 = 1538;
     int TICKS_FOR_THREE = 2307; // Ticks for 3 rotation is 384.5 * 6 = 2307;
     int TICKS_FOR_FOUR = 3076; // Ticks for 4 rotation is 384.5 * 8 = 3076;
     int TICKS_FOR_FIVE = 4150; // Ticks for 5 rotation is 384.5 * 10 = 3845;
     private int liftMotorTicks = TICKS_FOR_FOUR;
     private int armTicksHalfway = 550;
     */

    // RPM = 1620; TICKS_PER_REV = 103.8; Gear Ratio = 5;
    int TICKS_FOR_ONE = 519; // Ticks for 1 rotation is 103.8 * 5 = 58;
    int TICKS_FOR_TWO = 1038; // Ticks for 2 rotation is 28 * 10 = 1538;
    int TICKS_FOR_THREE = 1557; // Ticks for 3 rotation is 28 * 15 = 2307;
    private int liftMotorTicks = 425; //425;

//    private RobotDrive drive = new RobotDrive(this);
    RobotDrive drive = new RobotDrive();

    boolean driveSlow = false;
    boolean isLiftActivated = false;
    boolean isBottomReached = false;
    boolean isVertSensorTouchedOnce = false;
//    boolean isArmUp = false;
    boolean isArmActivatedHalfway = false;
    boolean isGrabberActive = false;
    boolean gamepad1XisPressed = false;
    boolean intakeActive = true;

//    boolean isWheelMotorRunning = false;

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
//        drive.deactivateArm();
        drive.restArm();
        drive.downLift();
        drive.activateIntakeServo();
//        drive.deactivateLeftRaiseClimb();
//        drive.deactivateRightRaiseClimb();
//        drive.deactivateLeftClimb();
//        drive.deactivateRightClimb();
//        drive.deactivateDroneServo();
        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            operateTeleOp();
        }
    }

    private void operateTeleOp() {
        double forward = gamepad1.left_stick_y * -1; //The y direction on the gamepad is reversed idk why
        double strafe = gamepad1.left_stick_x * 1;
        double rotate = gamepad1.right_stick_x * 1;

        double forward2 = gamepad2.left_stick_y * -1; //The y direction on the gamepad is reversed idk why
        double strafe2 = gamepad2.left_stick_x * 1;
        double rotate2 = gamepad2.right_stick_x * 1;

        drive.driveMecanum(forward + forward2, (strafe + strafe2)*(driveSlow?1.4:1), rotate + rotate2, driveSlow);

        //check lift sensors and decide whether to stop it or not...
        if(!isLiftActivated) {
            isBottomReached = drive.checkLiftMotion();
            if(isBottomReached) {
                isVertSensorTouchedOnce = true;
            }
        }

        if(gamepad1.left_bumper){
            drive.deactivateLeftClaw();
        }
        if(gamepad1.right_bumper) {
            drive.deactivateRightClaw();
        }
        if(gamepad1.a) {
            drive.activateLeftClaw();
            drive.activateRightClaw();
            drive.stopIntake();
            drive.deactivateIntakeServo();
        }

        if((gamepad1.x) && (isVertSensorTouchedOnce)){
            drive.activateIntake();
            drive.activateIntakeServo();
            drive.deactivateArm();
            drive.deactivateLeftClaw();
            drive.deactivateRightClaw();
        }
        if(gamepad1.y){
            drive.reverseIntake();
        }

        if ((gamepad2.dpad_left) && (isBottomReached || isVertSensorTouchedOnce)) {
            if (!isLiftActivated) {
                //drive.reverseIntake();
                drive.activateLift(550);
                drive.activateArm();
                drive.stopIntake();
                drive.deactivateIntakeServo();
//                isArmUp = true;
                drive.activateIntakeServo();
                driveSlow = true;
                isLiftActivated = true;
                isBottomReached = false;
                isVertSensorTouchedOnce = false;

            }
        }

        if ((gamepad2.a) && (isBottomReached || isVertSensorTouchedOnce)) {
            if (!isLiftActivated) {
                //drive.reverseIntake();
                drive.activateLift(400);
                drive.activateArm();
                drive.stopIntake();
//                isArmUp = true;
                drive.activateIntakeServo();
                driveSlow = true;
                isLiftActivated = true;
                isBottomReached = false;
                isVertSensorTouchedOnce = false;

            }
        }

        if ((gamepad2.dpad_up) && (isBottomReached || isVertSensorTouchedOnce)) {
            if (!isLiftActivated) {
                //drive.reverseIntake();
                drive.activateLift(850);
                drive.activateArm();
                drive.stopIntake();
//                isArmUp = true;
                drive.deactivateIntakeServo();
                driveSlow = true;
                isLiftActivated = true;
                isBottomReached = false;
                isVertSensorTouchedOnce = false;

            }
        }

        if ((gamepad2.dpad_right) && (isBottomReached || isVertSensorTouchedOnce)) {
            if (!isLiftActivated) {

                drive.activateLift(1250);
                drive.activateArm();
                drive.stopIntake();
//                isArmUp = true;
                drive.deactivateIntakeServo();
                driveSlow = true;
                isLiftActivated = true;
                isBottomReached = false;
                isVertSensorTouchedOnce = false;

            }
        }
        if(gamepad2.left_trigger > 0.5) {
            if (!isLiftActivated) {

                drive.activateLift(1400);
                drive.activateArm();
                drive.stopIntake();
//                isArmUp = true;
                drive.deactivateIntakeServo();
                driveSlow = true;
                isLiftActivated = true;
                isBottomReached = false;
                isVertSensorTouchedOnce = false;

            }
        }

        if(gamepad2.right_trigger > 0.5) {
            if (!isLiftActivated) {

                drive.activateLift(1600);
                drive.activateArm();
                drive.stopIntake();
//                isArmUp = true;
                drive.deactivateIntakeServo();
                driveSlow = true;
                isLiftActivated = true;
                isBottomReached = false;
                isVertSensorTouchedOnce = false;

            }
        }
        if (gamepad2.dpad_down) {
            if (isLiftActivated) {
                drive.deactivateLift();
                drive.restArm();
                drive.reverseIntake();
                drive.activateIntakeServo();
//                isArmUp = true;
                driveSlow = false;
                isLiftActivated = false;
            }
        }

        if (gamepad1.b && !bPressed) {
            if (!climbServoActivated) {
                climbServoActivated = true;
            } else {
                climbServoActivated = false;
            }
            bPressed = true;
        }

        if (!gamepad1.b && bPressed)
        {
            bPressed = false;
        }

        if (!climbServoActivated) {
            drive.deactivateLeftClimb();
            drive.deactivateRightClimb();
        } else {
            drive.activateLeftClimb();
            drive.activateRightClimb();
        }

        if(gamepad1.dpad_right) {
            drive.activateRightRaiseClimb();
            drive.activateLeftRaiseClimb();
            drive.deactivateIntakeServo();
            drive.stopIntake();
        }
        if(gamepad1.dpad_left) {
            drive.deactivateRightRaiseClimb();
            drive.deactivateLeftRaiseClimb();
            drive.deactivateIntakeServo();
            drive.stopIntake();
        }
        if(gamepad1.dpad_up) {
            drive.activateDroneServo();
            drive.deactivateIntakeServo();
            drive.stopIntake();
        }
        if(gamepad1.dpad_down) {
            drive.activateLeftRaiseClimbDrone();
            drive.activateRightRaiseClimbDrone();
            drive.deactivateIntakeServo();
            drive.stopIntake();
        }

        if (!overrideAutoClaw) {
            if (drive.leftPixelContained())
                drive.activateLeftClaw();
            if (drive.rightPixelContained())
                drive.activateRightClaw();
            if (drive.leftPixelContained() && drive.rightPixelContained()) {
                drive.stopIntake();
                if (Math.abs(drive.intakeServo.getPosition() - RobotConstants.INTAKE_POSITION_REST) > .01) {
                    drive.deactivateIntakeServo();
                }
            }
        }

//        if (gamepad2.y && !overrideButtonPressed)
//        {
//            overrideButtonPressed = true;
//            if (overrideAutoClaw)
//                overrideAutoClaw = false;
//            else {
//                overrideAutoClaw = true;
//                drive.deactivateRightClaw();
//                drive.deactivateLeftClaw();
//            }
//        }
//        if (!gamepad2.y)
//        {
//            overrideButtonPressed = false;
//        }

//        if(gamepad2.dpad_down) {
//            drive.restArm();
//            drive.downLift();
//            drive.activateIntakeServo();
//        }

        telemetry.addData("isBottomReached: ", isBottomReached);
        telemetry.addData("isLiftActivated: ", isLiftActivated);
        telemetry.addData("pixelCount: ", drive.getPixelCount());
        telemetry.addData("override auto claw: ", overrideAutoClaw);
//        telemetry.addData("isArmUp: ", isArmUp);
        telemetry.update();
    }

}
