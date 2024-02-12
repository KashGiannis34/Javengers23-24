package org.firstinspires.ftc.teamcode.ftc8468;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class RobotConstants {

    public static final float ARM_POSITION_ACTIVE = 1.0f;
    public static final float ARM_POSITION_REST = .47f;
    public static final float ARM_POSITION_AUTO = .54f;
    public static final float ARM_POSITION_INIT = .70f;
    public static final float ARM_POSITION_DEACTIVE= 0.31f;

    public static final float LEFT_CLAW_POSITION_ACTIVE = 0.28f;
//    public static final float LEFT_CLAW_POSITION_DEACTIVE= 0.68f;
    public static final float LEFT_CLAW_POSITION_DEACTIVE= 0.875f;

    public static final float RIGHT_CLAW_POSITION_ACTIVE = .83f;
//    public static final float RIGHT_CLAW_POSITION_DEACTIVE= 0.43f;
    public static final float RIGHT_CLAW_POSITION_DEACTIVE= 0.25f;




    public static final float LEFT_CLIMB_POSITION_ACTIVE = 0.0f;
    public static final float LEFT_CLIMB_POSITION_DEACTIVE= 1.0f;

    public static final float RIGHT_CLIMB_POSITION_ACTIVE = 0.0f;
    public static final float RIGHT_CLIMB_POSITION_DEACTIVE= 1.0f;

    public static final float RIGHT_RAISE_CLIMB_POSITION_ACTIVE = 0.55f;
    public static final float RIGHT_RAISE_CLIMB_POSITION_DRONE = 0.80f;
    public static final float RIGHT_RAISE_CLIMB_POSITION_DEACTIVE= 0.92f;

    public static final float LEFT_RAISE_CLIMB_POSITION_ACTIVE = 0.88f;
    public static final float LEFT_RAISE_CLIMB_POSITION_DRONE = 0.68f;
    public static final float LEFT_RAISE_CLIMB_POSITION_DEACTIVE= 0.53f;

    public static final float INTAKE_POSITION_FIVE = 0.77f;
    public static final float INTAKE_POSITION_FOUR = 0.81f;
    public static final float INTAKE_POSITION_ACTIVE = 0.8f;
    public static final float INTAKE_POSITION_REST = 0.59f;

//    public static final float DRONE_POSITION_ACTIVE = 0.75f;
//    public static final float DRONE_POSITION_REST = 0.32f;
    public static final float DRONE_POSITION_ACTIVE = 1.0f;
    public static final float DRONE_POSITION_REST = 0.55f;


    // Servo Positions...
//    public static final float INTAKE_SERVO_POSITION_ACTIVE = 0.0f;
//    public static final float INTAKE_SERVO_POSITION_REST = 0.5f;

    public static final float LEFT_LINKAGE_POSITION_ACTIVE = 0.20f;
    public static final float LEFT_LINKAGE_POSITION_REST = 0.70f;
    public static final float LEFT_LINKAGE_POSITION_HALF = 0.55f;
    public static final float LEFT_LINKAGE_POSITION_SHARED = 0.65f;

    public static final float RIGHT_LINKAGE_POSITION_ACTIVE = 0.62f;
    public static final float RIGHT_LINKAGE_POSITION_REST = 0.20f;
    public static final float RIGHT_LINKAGE_POSITION_HALF = 0.35f;
    public static final float RIGHT_LINKAGE_POSITION_SHARED = 0.25f;

    public static final float RIGHT_ARM_POSITION_ACTIVE = 0.62f;
    public static final float RIGHT_ARM_POSITION_REST = 0.11f;
    public static final float RIGHT_ARM_POSITION_HALF = 0.52f;
    public static final float RIGHT_ARM_POSITION_EXACTHALF = 0.32f;
    public static final float RIGHT_ARM_POSITION_LITTLE = 0.21f;

    public static final float LEFT_ARM_POSITION_ACTIVE = 0.10f;
    public static final float LEFT_ARM_POSITION_REST = 0.60f;
    public static final float LEFT_ARM_POSITION_HALF = 0.20f;
    public static final float LEFT_ARM_POSITION_EXACTHALF = 0.30f;
    public static final float LEFT_ARM_POSITION_LITTLE = 0.50f;

    public static final float DUMP_SERVO_POSITION_REST = 0.20f;
    public static final float DUMP_SERVO_POSITION_HALF = 0.65f;
    public static final float DUMP_SERVO_POSITION_ACTIVE = 0.95f;

    public static final float KICKER_POSITION_OUT = 0.75f;
    public static final float KICKER_POSITION_REST = 0.28f;
    public static final float KICKER_POSITION_SOFTOUT = 0.48f;
    public static final float KICKER_POSITION_CLOSE = 0.10f;

    //PowerPlay Servo Positions
    //PowerPlay Servo Positions
    public static final float HORIZ_FOURBAR_LEFT_POSITION_ACTIVE = 0.84f;
    public static final float HORIZ_FOURBAR_LEFT_POSITION_REST = 0.20f;
    public static final float HORIZ_FOURBAR_LEFT_POSITION_HALF = 0.41f;

    public static final float HORIZ_FOURBAR_RIGHT_POSITION_ACTIVE = 0.21f;
    public static final float HORIZ_FOURBAR_RIGHT_POSITION_REST = 0.85f;
    public static final float HORIZ_FOURBAR_RIGHT_POSITION_HALF = 0.58f;

    public static final float PIVOT_POSITION_REST = 0.00f;
    public static final float PIVOT_POSITION_ACTIVE = 1.0f;

    public static final float FRONT_CLAW_POSITION_REST = 0.50f;
    public static final float FRONT_CLAW_POSITION_ACTIVE = 0.68f;

    public static final float FRONT_TURRET_POSITION_REST = 0.46f;
    public static final float FRONT_TURRET_POSITION_LEFT = 0.8f;
    public static final float FRONT_TURRET_POSITION_RIGHT = 0.8f;

    public static final float LEFT_MAIN_TURRET_POSITION_REST = 0.31f;
    public static final float LEFT_MAIN_TURRET_POSITION_LEFT = 0.02f;
    public static final float LEFT_MAIN_TURRET_POSITION_RIGHT = 0.57f;
    public static final float LEFT_MAIN_TURRET_POSITION_RED = 0.16f;

    public static final float RIGHT_MAIN_TURRET_POSITION_REST = 0.45f;
    public static final float RIGHT_MAIN_TURRET_POSITION_LEFT = 0.19f;
    public static final float RIGHT_MAIN_TURRET_POSITION_RIGHT = 0.76f;
    public static final float RIGHT_MAIN_TURRET_POSITION_RED = 0.31f;

    public static final float LEFT_VERT_ARM_POSITION_REST = 0.85f;
    public static final float LEFT_VERT_ARM_POSITION_ACTIVE = 0.24f;
    public static final float LEFT_VERT_ARM_POSITION_PREP = 0.92f;
    public static final float LEFT_VERT_ARM_POSITION_GRAB = 0.82f;
    public static final float LEFT_VERT_ARM_POSITION_CAPSTONE = 0.00f;

    public static final float RIGHT_VERT_ARM_POSITION_REST = 0.03f;
    public static final float RIGHT_VERT_ARM_POSITION_ACTIVE = 0.75f;
    public static final float RIGHT_VERT_ARM_POSITION_PREP = 0.07f;
    public static final float RIGHT_VERT_ARM_POSITION_GRAB = 0.10f;
    public static final float RIGHT_VERT_ARM_POSITION_CAPSTONE = 1.00f;

    public static final float VERT_CLAW_POSITION_REST = 0.0f;
    public static final float VERT_CLAW_POSITION_ACTIVE = 0.26f;
    public static final float VERT_CLAW_POSITION_DEACTIVE = 0.10f;
    public static final float VERT_CLAW_POSITION_PREP = 0.08f;

    public static final float HORIZ_LOCK_ENGAGE= 0.54f;
    public static final float HORIZ_LOCK_DISENGAGE = 0.63f;
    // RPM = 435; TICKS PER REV = 384.5
    // RPM = 1150; TICKS_PER_REV = 145.1;
    // RPM = 1620; TICKS_PER_REV = 103.8;
    public static PIDFCoefficients VERT_VELO_PID = new PIDFCoefficients(2.0, 0, 0, (32767 / (1150 / 60 * 145.1)));

    public static PIDFCoefficients HORIZ_VELO_PID = new PIDFCoefficients(2.0, 0, 0, (32767 / (1150 / 60 * 145.1)));

}
