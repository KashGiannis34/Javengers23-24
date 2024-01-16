package org.firstinspires.ftc.teamcode.ftc8468.teleop.old;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.drive.opmode.virtualrobot.MecanumDrive;


public class RobotDriveOld extends MecanumDrive {

    private DcMotorEx intakeMotor;
    private DcMotorEx wheelMotor;

    private DcMotorEx liftMotorR;
    private DcMotorEx liftMotorL;

    protected Servo dumpServo;
    protected Servo intakeServoR;
    protected Servo intakeServoL;

    //Linkage
    protected Servo leftLinkage;
    protected Servo rightLinkage;

    //Arm
    protected Servo leftArm;
    protected Servo rightArm;

    protected Servo kickerServo;


    protected DigitalChannel liftSensorL;
    protected DigitalChannel liftSensorR;

    protected VoltageSensor batteryVoltageSensor;

    protected float speedMultiplier = 0.93f;
    final int LIFT_TOLERANCE = 0;

    // Servo Positions...
    final float INTAKE_SERVO_POSITION_ACTIVE = 0.0f;
    final float INTAKE_SERVO_POSITION_REST = 0.5f;

    final float LEFT_LINKAGE_POSITION_ACTIVE = 0.20f;
    final float LEFT_LINKAGE_POSITION_REST = 0.70f;
    final float LEFT_LINKAGE_POSITION_HALF = 0.55f;
    final float LEFT_LINKAGE_POSITION_SHARED = 0.65f;

    final float RIGHT_LINKAGE_POSITION_ACTIVE = 0.62f;
    final float RIGHT_LINKAGE_POSITION_REST = 0.20f;
    final float RIGHT_LINKAGE_POSITION_HALF = 0.35f;
    final float RIGHT_LINKAGE_POSITION_SHARED = 0.25f;

    final float RIGHT_ARM_POSITION_ACTIVE = 0.62f;
    final float RIGHT_ARM_POSITION_REST = 0.11f;
    final float RIGHT_ARM_POSITION_HALF = 0.52f;
    final float RIGHT_ARM_POSITION_EXACTHALF = 0.32f;
    final float RIGHT_ARM_POSITION_LITTLE = 0.21f;

    final float LEFT_ARM_POSITION_ACTIVE = 0.10f;
    final float LEFT_ARM_POSITION_REST = 0.60f;
    final float LEFT_ARM_POSITION_HALF = 0.20f;
    final float LEFT_ARM_POSITION_EXACTHALF = 0.30f;
    final float LEFT_ARM_POSITION_LITTLE = 0.50f;

    final float DUMP_SERVO_POSITION_REST = 0.20f;
    final float DUMP_SERVO_POSITION_HALF = 0.65f;
    final float DUMP_SERVO_POSITION_ACTIVE = 0.95f;

    final float KICKER_POSITION_OUT = 0.75f;
    final float KICKER_POSITION_REST = 0.28f;
    final float KICKER_POSITION_SOFTOUT = 0.48f;
    final float KICKER_POSITION_CLOSE = 0.10f;


    // RPM = 435; TICKS PER REV = 384.5
    //public static PIDFCoefficients LIFT_VELO_PID = new PIDFCoefficients(2.0, 0, 0, (32767 / (435 / 60 * 384.5)));
    // RPM = 1620; TICKS_PER_REV = 103.8;
    public static PIDFCoefficients LIFT_VELO_PID = new PIDFCoefficients(2.0, 0, 0, (32767 / (1620 / 60 * 103.8)));

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

        //dumpServo = hwMap.get(Servo.class, "dumpServo");
        //intakeServoR = hwMap.get(Servo.class, "intakeRight");
        //intakeServoR.setDirection(Servo.Direction.REVERSE);
        //intakeServoL = hwMap.get(Servo.class, "intakeLeft");
        leftLinkage = hwMap.get(Servo.class, "leftLinkage");
        rightLinkage = hwMap.get(Servo.class, "rightLinkage");

        leftArm = hwMap.get(Servo.class, "leftArm");
        rightArm = hwMap.get(Servo.class, "rightArm");

        kickerServo = hwMap.get(Servo.class, "kicker");

        liftSensorL = hwMap.get(DigitalChannel.class, "liftSensorL");
        liftSensorL.setMode(DigitalChannel.Mode.INPUT);
        liftSensorR = hwMap.get(DigitalChannel.class, "liftSensorR");
        liftSensorR.setMode(DigitalChannel.Mode.INPUT);

        intakeMotor = hwMap.get(DcMotorEx.class, "intakeMotor");
        liftMotorR = hwMap.get(DcMotorEx.class, "rightLift");
        liftMotorL = hwMap.get(DcMotorEx.class, "leftLift");

        wheelMotor = hwMap.get(DcMotorEx.class, "wheelMotor");

        batteryVoltageSensor = hwMap.voltageSensor.iterator().next();

        liftMotorR.setDirection(DcMotor.Direction.REVERSE);
        liftMotorR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotorL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(LIFT_VELO_PID != null) {
            setLiftPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LIFT_VELO_PID);
        }
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


    public void extendHorizSlide() {
        leftLinkage.setPosition(LEFT_LINKAGE_POSITION_ACTIVE);
        rightLinkage.setPosition(RIGHT_LINKAGE_POSITION_ACTIVE);
    }
    public void retractHorizSlide() {
        leftLinkage.setPosition(LEFT_LINKAGE_POSITION_REST);
        rightLinkage.setPosition(RIGHT_LINKAGE_POSITION_REST);
    }
    public void extendHorizSlideHalf() {
        leftLinkage.setPosition(LEFT_LINKAGE_POSITION_HALF);
        rightLinkage.setPosition(RIGHT_LINKAGE_POSITION_HALF);
    }
    public void extendHorizSlideShared() {
        leftLinkage.setPosition(LEFT_LINKAGE_POSITION_SHARED);
        rightLinkage.setPosition(RIGHT_LINKAGE_POSITION_SHARED);
    }

    public void flipArm() {
        leftArm.setPosition(LEFT_ARM_POSITION_ACTIVE);
        rightArm.setPosition(RIGHT_ARM_POSITION_ACTIVE);
    }
    public void flipArmHalf() {
        leftArm.setPosition(LEFT_ARM_POSITION_EXACTHALF);
        rightArm.setPosition(RIGHT_ARM_POSITION_EXACTHALF);
    }
    public void flipArmLittle() {
        leftArm.setPosition(LEFT_ARM_POSITION_LITTLE);
        rightArm.setPosition(RIGHT_ARM_POSITION_LITTLE);
    }
    public void restArm() {
        leftArm.setPosition(LEFT_ARM_POSITION_REST);
        rightArm.setPosition(RIGHT_ARM_POSITION_REST);
    }

    public void closeKicker() {
        kickerServo.setPosition(KICKER_POSITION_CLOSE);
    }
    public void restKicker() {
        kickerServo.setPosition(KICKER_POSITION_REST);
    }
    public void outKicker() {
        kickerServo.setPosition(KICKER_POSITION_OUT);
    }

    public void startIntake() {
        stopIntake();
        //deactivateDumpServo();
        intakeMotor.setPower(-1.0);
        //activateIntakeServo();
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
        //deactivateIntakeServo();
    }

    public void reverseIntake() {
        stopIntake();
        intakeMotor.setPower(1.0);
        //deactivateIntakeServo();
    }

    public void startWheelMotor(Double power) {
        stopWheelMotor();
        wheelMotor.setPower(power);
    }

    public void stopWheelMotor() {
        wheelMotor.setPower(0);
    }


    public void activateDumpServo() {

        dumpServo.setPosition(DUMP_SERVO_POSITION_ACTIVE);
    }

    public void activateDumpServoHalf() {

        dumpServo.setPosition(DUMP_SERVO_POSITION_HALF);
    }

    public void activateDumpServo(double position) {
        dumpServo.setPosition(position);
    }

    public void deactivateDumpServo() {

        dumpServo.setPosition(DUMP_SERVO_POSITION_REST);
    }

    public void activateIntakeServo() {
        intakeServoR.setPosition(INTAKE_SERVO_POSITION_ACTIVE);
        intakeServoL.setPosition(INTAKE_SERVO_POSITION_ACTIVE);
    }

    public void deactivateIntakeServo() {
        intakeServoR.setPosition(INTAKE_SERVO_POSITION_REST);
        intakeServoL.setPosition(INTAKE_SERVO_POSITION_REST);
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

    public void deactivateLift() {
        int ticks = 150;
        double power = -1.0; //-1.0;
        //deactivateDumpServo();

        liftMotorR.setTargetPosition(-ticks); // negative ticks for opposite direction
        liftMotorL.setTargetPosition(ticks); // negative ticks for opposite direction
        liftMotorR.setTargetPositionTolerance(LIFT_TOLERANCE);
        liftMotorL.setTargetPositionTolerance(LIFT_TOLERANCE);
        liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorR.setPower(power);
        liftMotorL.setPower(power);

        //startIntake();
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
        if( (liftSensorL.getState() == false) || (liftSensorR.getState() == false) ) {
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