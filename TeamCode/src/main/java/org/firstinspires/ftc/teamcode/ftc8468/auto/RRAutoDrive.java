package org.firstinspires.ftc.teamcode.ftc8468.auto;


import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ftc8468.RobotConstants;
import org.firstinspires.ftc.teamcode.ftc8468.teleop.RobotDrive;


public class RRAutoDrive extends SampleMecanumDrive {

    private DcMotorEx liftMotorL;
    private DcMotorEx liftMotorR;

    private DcMotorEx intakeMotor;

    protected Servo arm;
    protected Servo leftClaw;
    protected Servo rightClaw;

    protected Servo intakeServo;

    protected DigitalChannel horizSensorBottom;

    final int LIFT_TOLERANCE = 0;

    RevColorSensorV3 leftColorSensor, rightColorSensor;

    public enum PixelCount
    {
        ZERO,
        ONE,
        TWO
    }

    RRAutoDrive.PixelCount pixelCount = RRAutoDrive.PixelCount.ZERO;

    private ElapsedTime elapsedTime;


    ///////////////////////////
//    boolean isLiftActivated = false;
//    boolean isRetracting = false;
//
//    private DcMotorEx liftMotorL;
//    private DcMotorEx liftMotorR;
//
//    private DcMotorEx horizMotorL;
//    private DcMotorEx horizMotorR;
//
//    protected Servo horizFourbarL;
//    protected Servo horizFourbarR;
//    protected Servo pivotServo;
//    protected Servo frontClaw;
//    protected Servo frontTurret;
//
//    protected Servo leftTurret;
//    protected Servo rightTurret;
//
//    protected Servo leftVertFourbar;
//    protected Servo rightVertFourbar;
//
//    protected Servo vertClaw;
//
//    protected Servo horizLock;
//
//    protected DigitalChannel vertSensorL;
//    protected DigitalChannel vertSensorR;
//
//    protected DigitalChannel horizSensorL;
//    protected DigitalChannel horizSensorR;

    ////////////////////////////////////////
    protected float speedMultiplier = 0.6f;
    //    final int LIFT_TOLERANCE = 10;
    final int VERT_TOLERANCE = 0;
    final int HORIZ_TOLERANCE = 0;

    protected VoltageSensor batteryVoltageSensor;

//    public OpenCvWebcam webCamR;
//    public AprilTagDetectionPipeline pipeline;
//
//    //int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
//    int FIRST = 5;
//    int SECOND = 10;
//    int THIRD = 11;
//
//    public static final int CAMERA_WIDTH = 800; //1280;
//    public static final int CAMERA_HEIGHT = 448; //720;

//    protected float speedMultiplier = 0.6f;
////    final int LIFT_TOLERANCE = 10;
//    final int VERT_TOLERANCE = 0;
//    final int HORIZ_TOLERANCE = 0;

//    // Lens intrinsics
//    // UNITS ARE PIXELS
//    // NOTE: this calibration is for the C920 webcam at 800x448.
//    // You will need to do your own calibration for other configurations!
//    double fx = 578.272;
//    double fy = 578.272;
//    double cx = 402.145;
//    double cy = 221.506;
//
//    // UNITS ARE METERS
//    double tagsize = 0.166;
//    float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
//    int THRESHOLD_FRAMES_COUNT = 4;
//    final float DECIMATION_HIGH = 3;
//    final float DECIMATION_LOW = 2;
//    static final double FEET_PER_METER = 3.28084;


    // RPM = 435; TICKS PER REV = 384.5
    // RPM = 1150; TICKS_PER_REV = 145.1;
    // RPM = 1620; TICKS_PER_REV = 103.8;//
    public static PIDFCoefficients LIFT_VELO_PID = new PIDFCoefficients(2.0, 0, 0, (32767 / (1150 / 60 * 145.1)));

//    public static PIDFCoefficients HORIZ_VELO_PID = new PIDFCoefficients(2.0, 0, 0, (32767 / (1150 / 60 * 145.1)));

//    OpMode opMode;
//    RRAutoDrive() {
//        super();
//    }

//    RRAutoDrive(HardwareMap hwMap, OpMode _opMode) {
//        this(hwMap);
//        opMode = _opMode;
//    }

    public RRAutoDrive (HardwareMap hwMap) {
        super(hwMap);
        intakeMotor = hwMap.get(DcMotorEx.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm = hwMap.get(Servo.class, "armServo");

        leftClaw = hwMap.get(Servo.class, "leftClaw");
        rightClaw = hwMap.get(Servo.class, "rightClaw");

        intakeServo = hwMap.get(Servo.class, "intakeServo");

        horizSensorBottom = hwMap.get(DigitalChannel.class, "horizTouchBottom");
        horizSensorBottom.setMode(DigitalChannel.Mode.INPUT);

        batteryVoltageSensor = hwMap.voltageSensor.iterator().next();

        liftMotorL = hwMap.get(DcMotorEx.class, "liftMotorL");
        liftMotorR = hwMap.get(DcMotorEx.class, "liftMotorR");

        leftColorSensor = hwMap.get(RevColorSensorV3.class, "leftColorSensor");
        rightColorSensor = hwMap.get(RevColorSensorV3.class, "rightColorSensor");

        liftMotorR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorR.setDirection(DcMotor.Direction.REVERSE);
        liftMotorL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorL.setDirection(DcMotor.Direction.REVERSE);


        if(LIFT_VELO_PID != null) {
            setLiftPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LIFT_VELO_PID);
        }

        elapsedTime = new ElapsedTime();
        
//        super(hwMap);
//
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
//
//        batteryVoltageSensor = hwMap.voltageSensor.iterator().next();
//
//        liftMotorL = hwMap.get(DcMotorEx.class, "leftVert");
//        liftMotorR = hwMap.get(DcMotorEx.class, "rightVert");
//
//        horizMotorL = hwMap.get(DcMotorEx.class, "leftHoriz");
//        horizMotorR = hwMap.get(DcMotorEx.class, "rightHoriz");
//
//        liftMotorL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        liftMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        liftMotorR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        liftMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        if(LIFT_VELO_PID != null) {
//            setVertPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LIFT_VELO_PID);
//        }
//
//        horizMotorL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        horizMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        horizMotorR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        horizMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        if(HORIZ_VELO_PID != null) {
//            setHorizPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, HORIZ_VELO_PID);
//        }
//
//        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
//        webCamR = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "WebcamR"), cameraMonitorViewId);
    }

    // ***** FTC 18305 custom code *****

    //    void driveMecanum(double forward, double strafe, double rotate, boolean driveSlow) {
    //        double frontLeftSpeed = forward + strafe + rotate;
    //        double frontRightSpeed = forward - strafe - rotate;
    //        double backLeftSpeed = forward - strafe + rotate;
    //        double backRightSpeed = forward + strafe - rotate;
    //
    //        if(driveSlow) {
    //            frontLeftSpeed = frontLeftSpeed * speedMultiplier;
    //            frontRightSpeed = frontRightSpeed * speedMultiplier;
    //            backLeftSpeed = backLeftSpeed * speedMultiplier;
    //            backRightSpeed = backRightSpeed * speedMultiplier;
    //        }
    //
    //        setSpeeds(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
    //    }


    public void setLiftPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );
        liftMotorR.setPIDFCoefficients(runMode, compensatedCoefficients);
        liftMotorL.setPIDFCoefficients(runMode, compensatedCoefficients);
    }

//    public void setHorizPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
//        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
//                coefficients.p, coefficients.i, coefficients.d,
//                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
//        );
//        horizMotorR.setPIDFCoefficients(runMode, compensatedCoefficients);
//        horizMotorL.setPIDFCoefficients(runMode, compensatedCoefficients);
//    }

    /**
     * Starts camera and sets camera to pipeline.
     */
//    public void startCamera() {
//        Log.d("StartCamera", "Creating Camera");
//        if (webCamR != null) {
//            Log.d("startCamera", "Creating Pipeline");
//            pipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
//            webCamR.setPipeline(pipeline);
//            webCamR.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//                @Override
//                public void onOpened() {
//
//                    Log.d("StartCamera", "Creating Stream");
//                    webCamR.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN);
//                }
//
//                @Override
//                public void onError(int i) {
//
//                }
//            });
//        }
//        //FtcDashboard.getInstance().startCameraStream(webCamR, 10);
//        Log.d("startCamera", "Camera Initialized");
//    }

//    public int getImagePosition() {
//        int position = 0;
//        int framesCount = 0;
//        //AprilTagDetectionPipeline aprilTagPipeline = (AprilTagDetectionPipeline) pipeline;
//        ArrayList<AprilTagDetection> currentDetections = pipeline.getLatestDetections();
//        if(currentDetections != null) {
////            opMode.telemetry.addData("FPS", webCamR.getFps());
////            opMode.telemetry.addData("Overhead ms", webCamR.getOverheadTimeMs());
////            opMode.telemetry.addData("Pipeline ms", webCamR.getPipelineTimeMs());
//
//            if (currentDetections.size() == 0) {
//                framesCount++;
//                if(framesCount >= THRESHOLD_FRAMES_COUNT) {
//                    pipeline.setDecimation(DECIMATION_LOW);
//                }
//
//            } else {
//                framesCount = 0;
//
//                if(currentDetections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
//                    pipeline.setDecimation(DECIMATION_HIGH);
//                }
//
//                for (AprilTagDetection tag : currentDetections) {
////                    opMode.telemetry.addLine(String.format("\nDetected tag ID=%d", tag.id));
////                    opMode.telemetry.addLine(String.format("Translation X: %.2f feet", tag.pose.x*FEET_PER_METER));
////                    opMode.telemetry.addLine(String.format("Translation Y: %.2f feet", tag.pose.y*FEET_PER_METER));
////                    opMode.telemetry.addLine(String.format("Translation Z: %.2f feet", tag.pose.z*FEET_PER_METER));
////                    opMode.telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(tag.pose.yaw)));
////                    opMode.telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(tag.pose.pitch)));
////                    opMode.telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(tag.pose.roll)));
//
//                    if (tag.id == FIRST || tag.id == SECOND || tag.id == THIRD) {
//                        position = tag.id;
//                        if (tag.id == FIRST) {
//                            position = 1;
//                        } else if (tag.id == SECOND) {
//                            position = 2;
//                        } else if (tag.id == THIRD) {
//                            position = 3;
//                        }
//                        break;
//                    }
//                }
//            }
////            opMode.telemetry.update();
//        }
//        return position;
//    }

//    public void switchCameraToConePipeline() {
//        BlueConePipeline conePipeline = new BlueConePipeline();
//        pipeline = conePipeline;
//        webCamR.setPipeline(conePipeline);
//    }

/*
    public boolean alignIntakeTurret(Telemetry telemetry) {
        boolean isAligned = false;
        BlueConePipeline conePipeline = (BlueConePipeline) pipeline;

        double centerX = conePipeline.getCenterX();
        double threshold = 250;
        ElapsedTime timer = new ElapsedTime();
//        while (!isAligned) {
        if(conePipeline.isBlueVisible()) {
            double imageCenterX = conePipeline.getCenterofRect(conePipeline.getBlueRect()).x;
            double diff = centerX - imageCenterX;
            if(Math.abs(diff) > threshold) {
                if(diff > 0) {
//                    turretServo.setPosition(RobotConstants.turretServo.getPosition() - 0.0005);
                    intakeTurretServo.setPosition(RobotConstants.Range.clip(intakeTurretServo.getPosition() - .0005, 0, 1));
                } else if(diff < 0) {
//                    turretServo.setPosition(RobotConstants.turretServo.getPosition() + 0.0005);
                    intakeTurretServo.setPosition(RobotConstants.Range.clip(intakeTurretServo.getPosition() + .0005, 0, 1));
                }
            } else {
                isAligned = true;
            }
            telemetry.addData("intakeTurretServo position: ", intakeTurretServo.getPosition());
            telemetry.update();
        }
//        }
        return isAligned;
    }
*/


// *****   PowerPlay Functions   *****
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
//
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
//
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
//
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
//
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
//
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
//
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
//
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
//
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

    public void activateIntake() {
        intakeMotor.setPower(1);
    }
    public void deactivateIntake() {
        intakeMotor.setPower(0);
    }
    public void reverseIntake() {
        intakeMotor.setPower(-.8);
    }

    public void activateArm() {
        arm.setPosition(RobotConstants.ARM_POSITION_ACTIVE);
    }
    public void initArm() {
        arm.setPosition(RobotConstants.ARM_POSITION_INIT);
    }

    public void restArm() {
        arm.setPosition(RobotConstants.ARM_POSITION_REST);
    }
    public void restArmAuto() {
        arm.setPosition(RobotConstants.ARM_POSITION_AUTO);
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

    public void activateIntakeServo() {
        intakeServo.setPosition(RobotConstants.INTAKE_POSITION_ACTIVE_AUTO);
    }
    public void activateIntakeServoFive() {
        intakeServo.setPosition(RobotConstants.INTAKE_POSITION_FIVE);
    }
    public void activateIntakeServoFour() {
        intakeServo.setPosition(RobotConstants.INTAKE_POSITION_FOUR);
    }
    public void activateIntakeServoThree() {
        intakeServo.setPosition(RobotConstants.INTAKE_POSITION_THREE);
    }
    public void activateIntakeServoTwo() {
        intakeServo.setPosition(RobotConstants.INTAKE_POSITION_TWO);
    }

    public void activateIntakeServoOne() {
        intakeServo.setPosition(RobotConstants.INTAKE_POSITION_TELEOP);
    }
    public void deactivateIntakeServo() {
        intakeServo.setPosition(RobotConstants.INTAKE_POSITION_REST);
    }

    public double getIntakeServoPosition() {
        return intakeServo.getPosition();
    }

    public void setIntakeServoPosition(double pos) {
        intakeServo.setPosition(pos);
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

    public RRAutoDrive.PixelCount getPixelCount()
    {
        if (leftColorSensor.getDistance(DistanceUnit.CM) > 0.7 && rightColorSensor.getDistance(DistanceUnit.CM) > 0.7)
        {
            pixelCount = RRAutoDrive.PixelCount.ZERO;
        }
        else if (leftColorSensor.getDistance(DistanceUnit.CM) <= 0.7 && rightColorSensor.getDistance(DistanceUnit.CM) <= 0.7)
        {
            pixelCount = RRAutoDrive.PixelCount.TWO;
        }
        else
        {
            pixelCount = RRAutoDrive.PixelCount.ONE;
        }
        return pixelCount;
    }

    public boolean leftPixelContained()
    {
        if (leftColorSensor.getDistance(DistanceUnit.CM) <= 0.7)
            return true;
        else
            return false;
    }

    public boolean rightPixelContained()
    {
        if (rightColorSensor.getDistance(DistanceUnit.CM) <= 0.7)
            return true;
        else
            return false;
    }

    public void resetRuntime()
    {
        elapsedTime.reset();
    }

    public double elapsedMilliseconds()
    {
        return elapsedTime.milliseconds();
    }

    public double elapsedSeconds()
    {
        return elapsedTime.seconds();
    }

    public static TrajectoryVelocityConstraint velCon(double vel)
    {
        return SampleMecanumDrive.getVelocityConstraint(vel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    }

    public static TrajectoryAccelerationConstraint accCon(double acc)
    {
        return SampleMecanumDrive.getAccelerationConstraint(acc);
    }

}