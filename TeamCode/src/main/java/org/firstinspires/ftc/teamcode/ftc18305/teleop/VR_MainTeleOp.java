package org.firstinspires.ftc.teamcode.ftc18305.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "VR_MainTeleOp", group = "ftc18305")
public class VR_MainTeleOp extends LinearOpMode {
    private double wheelMotorSpeed = 0.7;
    private int shooterTicks = 2475;
    private int shooterTicksEndGame = 2375;

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
    boolean isArmUp = false;
    boolean isArmActivatedHalfway = false;
    boolean isGrabberActive = false;

//    boolean isWheelMotorRunning = false;

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
        drive.deactivateArm();
        drive.deactivateLift();
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

        drive.driveMecanum(forward + forward2, strafe + strafe2, rotate + rotate2, driveSlow);

        //check lift sensors and decide whether to stop it or not...
        if(!isLiftActivated) {
            isBottomReached = drive.checkLiftMotion();
            if(isBottomReached) {
                isVertSensorTouchedOnce = true;
            }
        }

        if(gamepad1.left_bumper){
            drive.deactivateLeftClaw();
            drive.deactivateRightClaw();
        }
        if(gamepad1.right_bumper) {
            drive.activateLeftClaw();
            drive.activateRightClaw();
            drive.deactivateIntake();
        }
        if(gamepad1.x){
            drive.activateArm();
        }
        if(gamepad1.a){
            drive.restArm();
            isArmUp = true;
        }
        if(gamepad1.b){
            if(!isLiftActivated) {
                drive.deactivateArm();
                isArmUp = false;
                drive.deactivateLeftClaw();
                drive.deactivateRightClaw();
            }
        }
        if(gamepad1.y){
            drive.activateIntake();
        }

        ////////////

        if ((gamepad1.dpad_up) && isArmUp && (isBottomReached || isVertSensorTouchedOnce)) {
            if (!isLiftActivated) {
                //drive.reverseIntake();
                drive.activateLift(liftMotorTicks);
                driveSlow = true;
                isLiftActivated = true;
                isBottomReached = false;
                isVertSensorTouchedOnce = false;

                //Optimization Commands
//                drive.closeKicker();
//                drive.extendHorizSlideHalf();
//                drive.flipArm();


            }
        }

        if (gamepad1.dpad_down) {
            if (isLiftActivated) {
                drive.deactivateLift();
                driveSlow = false;
                isLiftActivated = false;
            }
        }

        telemetry.addData("isBottomReached: ", isBottomReached);
        telemetry.addData("isLiftActivated: ", isLiftActivated);
        telemetry.addData("isArmUp: ", isArmUp);
        telemetry.update();
    }

}
