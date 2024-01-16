package org.firstinspires.ftc.teamcode.ftc18305.test.camera;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ftc18305.test.camera.sample.BlueConePipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class RobotDriveTemp extends SampleMecanumDrive {

    public static final int CAMERA_WIDTH = 1280; //320;
    public static final int CAMERA_HEIGHT = 720; //240;

    protected Servo turretServo;
    private DistanceSensor distanceSensor;
    public OpenCvWebcam webCamR;
    public BlueConePipeline pipeline;
    private DcMotorEx liftMotorR;
    protected VoltageSensor batteryVoltageSensor;

    // RPM = 435; TICKS_PER_REV = 384.5;
    public static PIDFCoefficients LIFT_VELO_PID = new PIDFCoefficients(2.0, 0, 0, (32767 / (435 / 60 * 384.5)));


    public RobotDriveTemp(HardwareMap hwMap) {
        super(hwMap);
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        if (webCamR == null) {
            webCamR = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "WebcamL"), cameraMonitorViewId);
            webCamR.setMillisecondsPermissionTimeout(50000);
        }

        turretServo = hwMap.get(Servo.class, "turretServo");
        distanceSensor = hwMap.get(DistanceSensor.class, "DistanceSensor1");

        batteryVoltageSensor = hwMap.voltageSensor.iterator().next();

        liftMotorR = hwMap.get(DcMotorEx.class, "liftMotor");
        liftMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotorR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(LIFT_VELO_PID != null) {
            setLiftPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LIFT_VELO_PID);
        }
    }

    public void setLiftPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );
        liftMotorR.setPIDFCoefficients(runMode, compensatedCoefficients);
    }

    /**
     * Starts camera and sets camera to pipeline.
     */
    public void startCamera() {
        Log.d("StartCamera", "Creating Camera");
        if (webCamR != null) {
            webCamR.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {

                    Log.d("StartCamera", "Creating Stream");
                    webCamR.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);


                    Log.d("startCamera", "Creating Pipeline");
                    pipeline = new BlueConePipeline();
                    webCamR.setPipeline(pipeline);
                }

                @Override
                public void onError(int i) {

                }
            });
        }
        //FtcDashboard.getInstance().startCameraStream(webCamR, 10);
        Log.d("startCamera", "Camera Initialized");
    }

    /**
     * Starts camera and sets camera to pipeline.
     */
    public void startCamera(OpenCvPipeline pipeline) {
        Log.d("StartCamera", "Creating Camera");
        if (webCamR != null) {
            webCamR.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {

                    Log.d("StartCamera", "Creating Stream");
                    webCamR.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);


                    Log.d("startCamera", "Creating Pipeline");
                    webCamR.setPipeline(pipeline);
                }

                @Override
                public void onError(int i) {

                }
            });
        }
        //FtcDashboard.getInstance().startCameraStream(webCamR, 10);
        Log.d("startCamera", "Camera Initialized");
    }

    public boolean alignTurret_BangBang() {
        boolean isAligned = false;
        double centerX = pipeline.getCenterX();
        double threshold = 250;
        ElapsedTime timer = new ElapsedTime();
//        while (!isAligned) {
            if(pipeline.isBlueVisible()) {
                double imageCenterX = pipeline.getCenterofRect(pipeline.getBlueRect()).x;
                double diff = centerX - imageCenterX;
                if(Math.abs(diff) > threshold) {
                    if(diff > 0) {
                        turretServo.setPosition(turretServo.getPosition() - .0005);
//                        turretServo.setPosition(Range.clip(turretServo.getPosition() - .01, 0, 1));
                    } else if(diff < 0) {
                        turretServo.setPosition(turretServo.getPosition() + .0005);
//                        turretServo.setPosition(Range.clip(turretServo.getPosition() + .01, 0, 1));
                    }
                } else {
                    isAligned = true;
                }
            }
//        }
        return isAligned;
    }

    public boolean alignTurret(Telemetry telemetry) {
        boolean isAligned = false;
        double centerX = pipeline.getCenterX();
        double threshold = 250;
        ElapsedTime timer = new ElapsedTime();
//        while (!isAligned) {
        if(pipeline.isBlueVisible()) {
            double imageCenterX = pipeline.getCenterofRect(pipeline.getBlueRect()).x;
            double diff = centerX - imageCenterX;
            if(Math.abs(diff) > threshold) {
                if(diff > 0) {
//                    turretServo.setPosition(turretServo.getPosition() - 0.0005);
                        turretServo.setPosition(Range.clip(turretServo.getPosition() - .0005, 0, 1));
                } else if(diff < 0) {
//                    turretServo.setPosition(turretServo.getPosition() + 0.0005);
                        turretServo.setPosition(Range.clip(turretServo.getPosition() + .0005, 0, 1));
                }
            } else {
                isAligned = true;
            }
            telemetry.addData("turretServo position: ", turretServo.getPosition());
            telemetry.update();
        }
//        }
        return isAligned;
    }

    public void activateLift(int ticks) {
        liftMotorR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        setLiftPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,LIFT_VELO_PID);

        double power = 1.0;
        liftMotorR.setTargetPosition(ticks);
        liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorR.setPower(power);

//        isLiftActivated = true;
//        isRetracting = false;
    }

    public void deactivateLift() {
        int ticks = 150;
        double power = -1.0;
        liftMotorR.setTargetPosition(-ticks); // negative ticks for opposite direction
        liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotorR.setPower(power);

//        isLiftActivated = false;
//        isRetracting = true;
    }

    public void stopLift() {
        liftMotorR.setPower(0);
    }

    public int[] checkAndMoveLift(int ticks, int tolerance) {
        int[] returnValues = new int[]{0, 0};
        double power = 0.5;
        int currentPosition = liftMotorR.getCurrentPosition();
        int diff = ticks - currentPosition;

        if(Math.abs(diff) > tolerance) {
            if ((diff > 0)) {
                liftMotorR.setPower(power);
            } else{
                liftMotorR.setPower(-power);
            }
        } else {
            liftMotorR.setPower(0);
            returnValues[1] = 1;
        }

//        if (distance > 10.0) {
//            if (Math.abs(diff) > tolerance) {
//                if ((diff > 0)) {
//                    liftMotorR.setPower(power);
//                } else {
//                    liftMotorR.setPower(-power);
//                }
//            } else {
//                liftMotorR.setPower(0);
//                returnValues[1] = 1;
//            }
//        }
        returnValues[0] = currentPosition;
        return returnValues;
    }

    public double getDistance() {
        double distance = 0.0;
        if(distanceSensor != null) {
            distance = distanceSensor.getDistance(DistanceUnit.CM);
        }
        return distance;
    }

}
