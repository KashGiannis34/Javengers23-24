package org.firstinspires.ftc.teamcode.ftc8468.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.drive.opmode.virtualrobot.MecanumDrive;
import org.firstinspires.ftc.teamcode.ftc8468.RobotConstants;


public class RobotDrive extends MecanumDrive {

    private DcMotorEx liftMotorL;
    private DcMotorEx liftMotorR;

    private DcMotorEx intakeMotor;

    protected Servo arm;
    protected Servo leftClaw;
    protected Servo rightClaw;

    protected Servo leftClimb;
    protected Servo rightClimb;

    protected Servo leftRaiseClimb;
    protected Servo rightRaiseClimb;

    protected Servo intakeServo;

    protected Servo droneServo;

    protected DigitalChannel horizSensorBottom;

    final int LIFT_TOLERANCE = 0;




//////////////////////////

    private DcMotorEx horizMotorL;
    private DcMotorEx horizMotorR;

    protected Servo horizFourbarL;
    protected Servo horizFourbarR;
    protected Servo pivotServo;
    protected Servo frontClaw;
    protected Servo frontTurret;

    protected Servo leftTurret;
    protected Servo rightTurret;

    protected Servo leftVertFourbar;
    protected Servo rightVertFourbar;

    protected Servo vertClaw;

    protected Servo horizLock;

    protected DigitalChannel vertSensorL;
    protected DigitalChannel vertSensorR;

    protected DigitalChannel horizSensorL;
    protected DigitalChannel horizSensorR;

    ///////////////////////////////////////////

    protected VoltageSensor batteryVoltageSensor;

    protected float speedMultiplier = 0.3f; //0.93f;
    final int VERT_TOLERANCE = 5;
//    final int HORIZ_TOLERANCE = 20;

    // Servo Positions...

    // RPM = 435; TICKS PER REV = 384.5
    // RPM = 1620; TICKS_PER_REV = 103.8;
    // RPM = 1150; TICKS_PER_REV = 145.1;
    public static PIDFCoefficients LIFT_VELO_PID = new PIDFCoefficients(2.0, 0, 0, (32767 / (1150 / 60 * 145.1)));

//    public static PIDFCoefficients HORIZ_VELO_PID = new PIDFCoefficients(2.0, 0, 0, (32767 / (1150 / 60 * 145.1)));

//    OpMode opMode;
//    RobotDrive() {
//        super();
//    }
//
//    RobotDrive(OpMode _opMode) {
//        super();
//        opMode = _opMode;
//    }

    public void init(HardwareMap hwMap) {
        super.init(hwMap);
        intakeMotor = hwMap.get(DcMotorEx.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm = hwMap.get(Servo.class, "armServo");

        leftClaw = hwMap.get(Servo.class, "leftClaw");
        rightClaw = hwMap.get(Servo.class, "rightClaw");

        leftClimb = hwMap.get(Servo.class, "leftClimb");
        rightClimb = hwMap.get(Servo.class, "rightClimb");

        leftRaiseClimb = hwMap.get(Servo.class, "leftRaiseClimb");
        rightRaiseClimb = hwMap.get(Servo.class, "rightRaiseClimb");

        intakeServo = hwMap.get(Servo.class, "intakeServo");

        droneServo = hwMap.get(Servo.class, "drone");

        horizSensorBottom = hwMap.get(DigitalChannel.class, "horizTouchBottom");
        horizSensorBottom.setMode(DigitalChannel.Mode.INPUT);



//        horizFourbarL = hwMap.get(Servo.class, "leftHorizFourbar");
//        horizFourbarR = hwMap.get(Servo.class, "rightHorizFourbar");
//        pivotServo = hwMap.get(Servo.class, "pivot");
//        frontClaw = hwMap.get(Servo.class, "frontClaw");
//        frontTurret = hwMap.get(Servo.class, "frontTurret");
//
//        leftTurret = hwMap.get(Servo.class, "leftTurret");
//        rightTurret = hwMap.get(Servo.class, "rightTurret");
//
//        leftVertFourbar = hwMap.get(Servo.class, "leftVertFourbar");
//        rightVertFourbar = hwMap.get(Servo.class, "leftVertFourbar");
//
//        vertClaw = hwMap.get(Servo.class, "vertClaw");
//
//        horizLock = hwMap.get(Servo.class, "horizLock");
//
//        vertSensorL = hwMap.get(DigitalChannel.class, "leftVertTouch");
//        vertSensorL.setMode(DigitalChannel.Mode.INPUT);
//        vertSensorR = hwMap.get(DigitalChannel.class, "rightVertTouch");
//        vertSensorR.setMode(DigitalChannel.Mode.INPUT);
//
//        horizSensorL = hwMap.get(DigitalChannel.class, "leftHorizTouch");
//        horizSensorL.setMode(DigitalChannel.Mode.INPUT);
//        horizSensorR = hwMap.get(DigitalChannel.class, "rightHorizTouch");
//        horizSensorR.setMode(DigitalChannel.Mode.INPUT);

        batteryVoltageSensor = hwMap.voltageSensor.iterator().next();

        liftMotorL = hwMap.get(DcMotorEx.class, "liftMotorL");
        liftMotorR = hwMap.get(DcMotorEx.class, "liftMotorR");

        liftMotorR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorR.setDirection(DcMotor.Direction.REVERSE);
        liftMotorL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorL.setDirection(DcMotor.Direction.REVERSE);


        if(LIFT_VELO_PID != null) {
            setLiftPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LIFT_VELO_PID);
        }

//        horizMotorL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        horizMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        horizMotorR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        horizMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        if(HORIZ_VELO_PID != null) {
//            setHorizPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, HORIZ_VELO_PID);
//        }

    }

    // ***** FTC 18305 custom code *****

    void driveMecanum(double forward, double strafe, double rotate, boolean driveSlow) {
        double frontLeftSpeed = forward + strafe + rotate;
        double frontRightSpeed = forward - strafe - rotate;
        double backLeftSpeed = forward - strafe + rotate;
        double backRightSpeed = forward + strafe - rotate;

        if(driveSlow) {
            frontLeftSpeed = frontLeftSpeed * speedMultiplier;
            frontRightSpeed = frontRightSpeed * speedMultiplier;
            backLeftSpeed = backLeftSpeed * speedMultiplier;
            backRightSpeed = backRightSpeed * speedMultiplier;
        }

        setSpeeds(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
    }

    public void setLiftPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );
        liftMotorR.setPIDFCoefficients(runMode, compensatedCoefficients);
        liftMotorL.setPIDFCoefficients(runMode, compensatedCoefficients);
    }

//        public void setHorizPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
//        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
//                coefficients.p, coefficients.i, coefficients.d,
//                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
//        );
//        horizMotorR.setPIDFCoefficients(runMode, compensatedCoefficients);
//        horizMotorL.setPIDFCoefficients(runMode, compensatedCoefficients);
//    }

// ***** Common Drive operations *****

//    public void activatePivot() {
//        pivotServo.setPosition(RobotConstants.PIVOT_POSITION_ACTIVE);
//    }
//    public void deactivatePivot() {
//        pivotServo.setPosition(RobotConstants.PIVOT_POSITION_REST);
//    }
//
//    public void activateFrontClaw() {
//        frontClaw.setPosition(RobotConstants.FRONT_CLAW_POSITION_ACTIVE);
//    }
//    public void deactivateFrontClaw() {
//        frontClaw.setPosition(RobotConstants.FRONT_CLAW_POSITION_REST);
//    }
//
//    public void activateHorizFourbar() {
//        horizFourbarL.setPosition(RobotConstants.HORIZ_FOURBAR_LEFT_POSITION_ACTIVE);
//        horizFourbarR.setPosition(RobotConstants.HORIZ_FOURBAR_RIGHT_POSITION_ACTIVE);
//    }
//    public void activateHorizFourbarHalf() {
//        horizFourbarL.setPosition(RobotConstants.HORIZ_FOURBAR_LEFT_POSITION_HALF);
//        horizFourbarR.setPosition(RobotConstants.HORIZ_FOURBAR_RIGHT_POSITION_HALF);
//    }
//    public void deactivateHorizFourbar() {
//        horizFourbarL.setPosition(RobotConstants.HORIZ_FOURBAR_LEFT_POSITION_REST);
//        horizFourbarR.setPosition(RobotConstants.HORIZ_FOURBAR_RIGHT_POSITION_REST);
//    }
//
//    public void adjustHorizFourbar(boolean moveUp) {
//        if(moveUp) {
//            horizFourbarL.setPosition(Range.clip(horizFourbarL.getPosition() - .01, 0, 1));
//            horizFourbarR.setPosition(Range.clip(horizFourbarR.getPosition() + .01, 0, 1));
//        } else {
//            horizFourbarL.setPosition(Range.clip(horizFourbarL.getPosition() + .01, 0, 1));
//            horizFourbarR.setPosition(Range.clip(horizFourbarR.getPosition() - .01, 0, 1));
//        }
//    }
//
//    public void restFrontTurret() {
//        frontTurret.setPosition(RobotConstants.FRONT_TURRET_POSITION_REST);
//    }

//    public void restMainTurret(){
//        leftTurret.setPosition(RobotConstants.LEFT_MAIN_TURRET_POSITION_REST);
//        rightTurret.setPosition(RobotConstants.RIGHT_MAIN_TURRET_POSITION_REST);
//    }
//
//    public void activateMainTurretRight(){
//        leftTurret.setPosition(RobotConstants.LEFT_MAIN_TURRET_POSITION_RIGHT);
//        rightTurret.setPosition(RobotConstants.RIGHT_MAIN_TURRET_POSITION_RIGHT);
//    }
//
//    public void activateMainTurretLeft(){
//        leftTurret.setPosition(RobotConstants.LEFT_MAIN_TURRET_POSITION_LEFT);
//        rightTurret.setPosition(RobotConstants.RIGHT_MAIN_TURRET_POSITION_LEFT);
//    }
//
//    public void activateMainTurretRed(){
//        leftTurret.setPosition(RobotConstants.LEFT_MAIN_TURRET_POSITION_RED);
//        rightTurret.setPosition(RobotConstants.RIGHT_MAIN_TURRET_POSITION_RED);
//    }
//
//    public void activateVertFourbar(){
//        rightVertFourbar.setPosition(RobotConstants.RIGHT_VERT_ARM_POSITION_ACTIVE);
//        leftVertFourbar.setPosition(RobotConstants.LEFT_VERT_ARM_POSITION_ACTIVE);
//    }

//    public void activateVertFourbarPrep(){
//        rightVertFourbar.setPosition(RobotConstants.RIGHT_VERT_ARM_POSITION_PREP);
//        leftVertFourbar.setPosition(RobotConstants.LEFT_VERT_ARM_POSITION_PREP);
//    }
//    public void activateVertFourbarGrab(){
//        rightVertFourbar.setPosition(RobotConstants.RIGHT_VERT_ARM_POSITION_GRAB);
//        leftVertFourbar.setPosition(RobotConstants.LEFT_VERT_ARM_POSITION_GRAB);
//    }
//    public void restVertFourbar(){
//        rightVertFourbar.setPosition(RobotConstants.RIGHT_VERT_ARM_POSITION_REST);
//        leftVertFourbar.setPosition(RobotConstants.LEFT_VERT_ARM_POSITION_REST);
//    }
//
//    public void activateVertClaw(){
//        vertClaw.setPosition(RobotConstants.VERT_CLAW_POSITION_ACTIVE);
//    }
//
//    public void deactivateVertClaw(){
//        vertClaw.setPosition(RobotConstants.VERT_CLAW_POSITION_DEACTIVE);
//    }
//    public void prepVertClaw(){
//        vertClaw.setPosition(RobotConstants.VERT_CLAW_POSITION_PREP);
//    }

//    public void restVertClaw(){
//        vertClaw.setPosition(RobotConstants.VERT_CLAW_POSITION_REST);
//    }
//
//    public void engageHorizLock(){horizLock.setPosition(RobotConstants.HORIZ_LOCK_ENGAGE);}
//
//    public void disengageHorizLock(){horizLock.setPosition(RobotConstants.HORIZ_LOCK_DISENGAGE);}
//
//
//    public void activateVertSlide(int ticks) {
//        liftMotorR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        liftMotorL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        setLiftPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,LIFT_VELO_PID);
//        double power = 1.0; //1.0;
//        liftMotorR.setTargetPosition(-ticks);
//        liftMotorL.setTargetPosition(ticks);
//        liftMotorR.setTargetPositionTolerance(VERT_TOLERANCE);
//        liftMotorL.setTargetPositionTolerance(VERT_TOLERANCE);
//        liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        liftMotorR.setPower(power);
//        liftMotorL.setPower(power);
//    }

//    public void activateHorizSlide(int ticks) {
//        horizMotorR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        horizMotorL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        setHorizPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,HORIZ_VELO_PID);
//        double power = 0.7; //1.0;
//        horizMotorR.setTargetPosition(ticks);
//        horizMotorL.setTargetPosition(-ticks);
//        horizMotorR.setTargetPositionTolerance(HORIZ_TOLERANCE);
//        horizMotorL.setTargetPositionTolerance(HORIZ_TOLERANCE);
//        horizMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        horizMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        horizMotorR.setPower(power);
//        horizMotorL.setPower(power);
//    }

//    public void deactivateVertSlide() {
//        int ticks = 150; //10;
//        double power = -0.5; //-1.0;
//
//        liftMotorR.setTargetPosition(ticks); // negative ticks for opposite direction
//        liftMotorL.setTargetPosition(-ticks); // negative ticks for opposite direction
//        liftMotorR.setTargetPositionTolerance(VERT_TOLERANCE);
//        liftMotorL.setTargetPositionTolerance(VERT_TOLERANCE);
//        liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        liftMotorR.setPower(power);
//        liftMotorL.setPower(power);
//    }

//    public void deactivateHorizSlide() {
//        int ticks = 150;
//        double power = -1.0; //-1.0;
//
//        horizMotorR.setTargetPosition(-ticks); // negative ticks for opposite direction
//        horizMotorL.setTargetPosition(ticks); // negative ticks for opposite direction
//        horizMotorR.setTargetPositionTolerance(HORIZ_TOLERANCE);
//        horizMotorL.setTargetPositionTolerance(HORIZ_TOLERANCE);
//        horizMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        horizMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        horizMotorR.setPower(power);
//        horizMotorL.setPower(power);
//    }

//    public void resetHorizSlide(int ticks) {
////        int ticks = 150;
//        double power = -1.0; //-1.0;
//
//        horizMotorR.setTargetPosition(-ticks); // negative ticks for opposite direction
//        horizMotorL.setTargetPosition(ticks); // negative ticks for opposite direction
//        horizMotorR.setTargetPositionTolerance(HORIZ_TOLERANCE);
//        horizMotorL.setTargetPositionTolerance(HORIZ_TOLERANCE);
//        horizMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        horizMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        horizMotorR.setPower(power);
//        horizMotorL.setPower(power);
//    }
//
//    public void stopVertSlide() {
//        liftMotorL.setPower(0);
//        liftMotorR.setPower(0);
//    }
//
//    public void stopHorizSlide() {
//        horizMotorL.setPower(0);
//        horizMotorR.setPower(0);
//    }

//    public void holdHorizSlide() {
//        horizMotorL.setPower(-0.1);
//        horizMotorR.setPower(-0.1);
//    }
//
//    public boolean checkVertMotion() {
//        boolean isBottomReached = false;
//        if (isVertSensorTouched()) {
//            stopVertSlide();
//            isBottomReached = true;
//        }
//        return isBottomReached;
//    }

//    public boolean isVertSensorTouched() {
//        // ***** if state == true, then "Not Pressed", Otherwise "Pressed" *****
////        if (sensor1.getState() == true) {
////            telemetry.addData("Digital Touch", "Is Not Pressed");
////        } else {
////            telemetry.addData("Digital Touch", "Is Pressed");
////        }
//
//        boolean isTouched = false;
//        if( (vertSensorL.getState() == false) || (vertSensorR.getState() == false) ) {
//            isTouched = true;
//        }
//        return isTouched;
//    }

//    public boolean checkHorizMotion() {
//        boolean isRetracted = false;
//        if (isHorizSensorTouched()) {
////            holdHorizSlide();
//            stopHorizSlide();
//            engageHorizLock();
//            deactivateHorizFourbar();
//            isRetracted = true;
//        }
//        return isRetracted;
//    }

//    public boolean isHorizSensorTouched() {
//        // ***** if state == true, then "Not Pressed", Otherwise "Pressed" *****
////        if (sensor1.getState() == true) {
////            telemetry.addData("Digital Touch", "Is Not Pressed");
////        } else {
////            telemetry.addData("Digital Touch", "Is Pressed");
////        }
//
//        boolean isTouched = false;
//        if( (horizSensorL.getState() == false) || (horizSensorR.getState() == false) ) {
//            isTouched = true;
//        }
//        return isTouched;
//    }

    //////////////////////////////////////////

    public void activateIntake() {
        intakeMotor.setPower(.9);
    }
    public void reverseIntake() {
        intakeMotor.setPower(-1.0);
    }
    public void stopIntake() {
        intakeMotor.setPower(0);
    }

    public void activateArm() {
        arm.setPosition(RobotConstants.ARM_POSITION_ACTIVE);
    }
    public void restArm() {
        arm.setPosition(RobotConstants.ARM_POSITION_REST);
    }
    public void deactivateArm() {
        arm.setPosition(RobotConstants.ARM_POSITION_DEACTIVE);
    }

    public void activateLeftClaw() {
        leftClaw.setPosition(RobotConstants.LEFT_CLAW_POSITION_ACTIVE);
    }
    public void deactivateLeftClaw() {
        leftClaw.setPosition(RobotConstants.LEFT_CLAW_POSITION_DEACTIVE);
    }

    public void activateRightClaw() {
        rightClaw.setPosition(RobotConstants.RIGHT_CLAW_POSITION_ACTIVE);
    }
    public void deactivateRightClaw() {
        rightClaw.setPosition(RobotConstants.RIGHT_CLAW_POSITION_DEACTIVE);
    }


    public void activateLeftClimb() {
        leftClimb.setPosition(RobotConstants.LEFT_CLIMB_POSITION_ACTIVE);
    }
    public void activateRightClimb() {
        rightClimb.setPosition(RobotConstants.RIGHT_CLIMB_POSITION_ACTIVE);
    }

    public void deactivateLeftClimb() {
        leftClimb.setPosition(RobotConstants.LEFT_CLIMB_POSITION_DEACTIVE);
    }
    public void deactivateRightClimb() {
        rightClimb.setPosition(RobotConstants.RIGHT_CLIMB_POSITION_DEACTIVE);
    }

    public void activateLeftRaiseClimb() {
        leftRaiseClimb.setPosition(RobotConstants.LEFT_RAISE_CLIMB_POSITION_ACTIVE);
    }

    public void activateLeftRaiseClimbDrone() {
        leftRaiseClimb.setPosition(RobotConstants.LEFT_RAISE_CLIMB_POSITION_DRONE);
    }
    public void activateRightRaiseClimb() {
        rightRaiseClimb.setPosition(RobotConstants.RIGHT_RAISE_CLIMB_POSITION_ACTIVE);
    }
    public void deactivateLeftRaiseClimb() {
        leftRaiseClimb.setPosition(RobotConstants.LEFT_RAISE_CLIMB_POSITION_DEACTIVE);
    }
    public void deactivateRightRaiseClimb() {
        rightRaiseClimb.setPosition(RobotConstants.RIGHT_RAISE_CLIMB_POSITION_DEACTIVE);
    }
    public void activateIntakeServo() {
        intakeServo.setPosition(RobotConstants.INTAKE_POSITION_ACTIVE);
    }
    public void deactivateIntakeServo() {
        intakeServo.setPosition(RobotConstants.INTAKE_POSITION_REST);
    }

    public void activateDroneServo() {
        droneServo.setPosition(RobotConstants.DRONE_POSITION_ACTIVE);
    }
    public void deactivateDroneServo() {
        droneServo.setPosition(RobotConstants.DRONE_POSITION_REST);
    }
    public void activateLift(int ticks) {
        // activateDumpServoHalf();
        liftMotorR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        setLiftPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,LIFT_VELO_PID);

        double power = 1.0; //1.0;
        liftMotorR.setTargetPosition(ticks);
        liftMotorL.setTargetPosition(-ticks);
        liftMotorR.setTargetPositionTolerance(LIFT_TOLERANCE);
        liftMotorL.setTargetPositionTolerance(LIFT_TOLERANCE);
        liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorR.setPower(power);
        liftMotorL.setPower(power);
    }

    public void downLift() {
        int ticks = 1000;
        double power = -1.0; //-1.0;
        liftMotorR.setTargetPosition(-ticks); // negative ticks for opposite direction
        liftMotorL.setTargetPosition(ticks); // negative ticks for opposite direction
        liftMotorR.setTargetPositionTolerance(LIFT_TOLERANCE);
        liftMotorL.setTargetPositionTolerance(LIFT_TOLERANCE);
        liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorR.setPower(power);
        liftMotorL.setPower(power);
    }

    public void deactivateLift() {
        int ticks = 150;
        double power = -1.0; //-1.0;
        liftMotorR.setTargetPosition(-ticks); // negative ticks for opposite direction
        liftMotorL.setTargetPosition(ticks); // negative ticks for opposite direction
        liftMotorR.setTargetPositionTolerance(LIFT_TOLERANCE);
        liftMotorL.setTargetPositionTolerance(LIFT_TOLERANCE);
        liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorR.setPower(power);
        liftMotorL.setPower(power);
    }

    public void stopLift() {
        liftMotorL.setPower(0);
        liftMotorR.setPower(0);
    }

    public boolean isLiftSensorTouched() {
        // ***** if state == true, then "Not Pressed", Otherwise "Pressed" *****
//        if (sensor1.getState() == true) {
//            telemetry.addData("Digital Touch", "Is Not Pressed");
//        } else {
//            telemetry.addData("Digital Touch", "Is Pressed");
//        }

        boolean isTouched = false;
        if(horizSensorBottom.getState() == false) {
            isTouched = true;
        }
        return isTouched;
    }

    public boolean checkLiftMotion() {
        boolean isBottomReached = false;
        if (isLiftSensorTouched()) {
            stopLift();
            //restArm();
            //restKicker();
            //startIntake();
            isBottomReached = true;
        }
        return isBottomReached;
    }

}