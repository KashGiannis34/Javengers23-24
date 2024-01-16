package org.firstinspires.ftc.teamcode.ftc18305.teleop.old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "VR_MainTeleOp_Old", group = "ftc18305")
public class VR_MainTeleOp_Old extends LinearOpMode {
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
    private int liftMotorTicks = 1650;

//    private RobotDriveOld drive = new RobotDriveOld(this);
    RobotDriveOld drive = new RobotDriveOld();

//    int intakeForwardCount = 1;
//    int intakeReverseCount = 1;
//    boolean isForward = false;
//    boolean isReverse = false;
//    boolean isGrabberOn = false;
//    boolean isLiftActivated = false;
    boolean driveSlow = false;
    boolean isLiftActivated = false;
    boolean isBottomReached = false;
    boolean isArmUp = false;
    boolean isArmActivatedHalfway = false;
    boolean isGrabberActive = false;

//    boolean isWheelMotorRunning = false;

    @Override
    public void runOpMode() throws InterruptedException {

        drive.init(hardwareMap);
        //drive.deactivateLift();
        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            operateTeleOp();
        }
    }

    private void operateTeleOp() {
        double forward = gamepad1.left_stick_y * 1; //The y direction on the gamepad is reversed idk why
        double strafe = gamepad1.left_stick_x * -1;
        double rotate = gamepad1.right_stick_x * -1;

        double forward2 = gamepad2.left_stick_y * 1; //The y direction on the gamepad is reversed idk why
        double strafe2 = gamepad2.left_stick_x * -1;
        double rotate2 = gamepad2.right_stick_x * -1;

        drive.driveMecanum(forward + forward2, strafe + strafe2, rotate + rotate2, driveSlow);


       //check lift sensors and decide whether to stop it or not...
        if(!isLiftActivated) {
            isBottomReached = drive.checkLiftMotion();
        }

        //Linkage buttons
        if (gamepad1.a || gamepad2.a) {
            drive.closeKicker();
            drive.flipArmLittle();
            isArmUp = true;
        }
        if (gamepad1.b || gamepad2.b) {
            //make sure Horiz Slide is retracted fully before resting the Arm!
            drive.restKicker();
            drive.retractHorizSlide();
            drive.restArm();
            isArmUp = false;
        }

        //Conditionals for Intake
        if (gamepad1.right_bumper || gamepad2.right_bumper) {
            drive.restKicker();
            drive.retractHorizSlide();
            drive.restArm();
            drive.startIntake();
        }

        if (gamepad1.left_bumper || gamepad2.left_bumper) {
            drive.reverseIntake();
        }

        if(gamepad1.x || gamepad2.x) {
            drive.outKicker();
        }

        if (gamepad1.y) {
            drive.stopIntake();
            drive.stopWheelMotor();
            
        }

        if ((gamepad1.dpad_up) && isArmUp && isBottomReached) {
            if (!isLiftActivated) {
                //drive.reverseIntake();
                drive.activateLift(liftMotorTicks);
                driveSlow = true;
                isLiftActivated = true;

                //Optimization Commands
                drive.closeKicker();
                drive.extendHorizSlideHalf();
                drive.flipArm();


            }
        }
        if ((gamepad1.dpad_left)) {
            drive.flipArmHalf();
        }

        if (gamepad1.dpad_down) {
            if (isLiftActivated) {
                drive.deactivateLift();
                driveSlow = false;
                isLiftActivated = false;

                //Optimization Commands
                drive.outKicker();
                drive.flipArmHalf();
                drive.stopIntake();
            }
        }
        if (gamepad2.dpad_up) {
            drive.flipArm();
            drive.extendHorizSlideShared();
            drive.reverseIntake();
        }


        if(gamepad1.left_trigger > 0.5) {
            drive.startWheelMotor(0.6);
        }

        if(gamepad1.right_trigger > 0.5) {
            drive.startWheelMotor(-0.6);
        }

//        if (gamepad2.dpad_up) {
//            if (!isLiftActivated) {
//                drive.reverseIntake();
//                drive.activateLift(TICKS_FOR_THREE);
//                driveSlow = true;
//                isLiftActivated = true;
//            }
//        }

/**
        if(gamepad1.x){
            drive.activateDumpServoHalf();
        }

        if(gamepad1.a){
            drive.activateDumpServo();
        }

        if(gamepad1.b){
            drive.deactivateDumpServo();
        }

        if(gamepad2.x){
            drive.startWheelMotor(wheelMotorSpeed);
        }

        if(gamepad2.y){
            drive.stopWheelMotor();
        }

        if(gamepad2.a){
            drive.activateDumpServo(0.88);
            sleep(400);
            drive.activateDumpServo(0.55);
        }

        if(gamepad2.b){
            drive.activateDumpServo(0.85);
        }
*/
    }

//    private void sleep(long millis) {
//        try {
//            Thread.sleep(millis);
//        } catch(InterruptedException e) {}
//    }

}
