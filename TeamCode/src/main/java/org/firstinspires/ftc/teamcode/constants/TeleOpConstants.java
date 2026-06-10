package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class TeleOpConstants {

    /* ========== DRIVETRAIN ========== */
    public static class Drivetrain {
        public static String FRONT_LEFT_MOTOR_NAME = "frontLeft";
        public static String FRONT_RIGHT_MOTOR_NAME = "frontRight";
        public static String BACK_LEFT_MOTOR_NAME = "backLeft";
        public static String BACK_RIGHT_MOTOR_NAME = "backRight";
    }
@Configurable
    /* ========== FLYWHEEL ========== */
    public static class Flywheel {
        public static String RIGHT_FLYWHEEL_MOTOR_NAME = "flywheel1";
        public static String LEFT_FLYWHEEL_MOTOR_NAME = "flywheel2";

        public static double FLYWHEEL_DIAMETER_INCHES = 3.0;
        public static double FLYWHEEL_HEIGHT_INCHES = 12.38335;
        public static double HOOD_ANGLE_DEGREES = 55;


        public static double KP = 0.001;
        public static double KV = 0.000341;
        public static double KS = 0.13;

        public static double FAR_VEL = 1800;
        public static double CLOSE_VEL = 1500;
        public static double VELOCITY_TOLERANCE = 10;
    }
    @Configurable
    /* ========== INTAKE ========== */
    public static class Intake{
        public static String INTAKE_MOTOR_NAME = "intake";
        public static String GATE_SERVO_NAME = "gate";

        public static double POWER = 0.8;

        public static double GATE_OPEN = 0.05;
        public static double GATE_CLOSE = 0.25;

    }

    @Configurable
    /* ========== HOOD ========== */
    public static class Hood{
        public static String HOOD_SERVO_NAME = "hood";

        public static final double[][] LOOKUP_TABLE = {

                {24, 0.05},
                {36, 0.10},
                {48, 0.16},
                {60, 0.23},
                {72, 0.30},
                {84, 0.38},
                {96, 0.46},
                {108, 0.54},
                {120, 0.63}

        };


    }


    @Configurable
    /* ========== TURRET ========== */
    public static class Turret {
        public static final double BLUE_TARGET_X = 0, BLUE_TARGET_Y =144;
        public static final double RED_TARGET_X = 144, RED_TARGET_Y =144;

        public static final double BLUE_START_X = 123.546, BLUE_START_Y =122.109, BLUE_START_HEADING = 37;

        public static final double RED_START_X = 123.546, RED_START_Y =122.109, RED_START_HEADING = 37;


        public static final String TURRET_MOTOR_NAME = "turret";

        public static final double TICKS_PER_MOTOR_REV = 435;

        public static final double GEAR_RATIO = 3.0;

        public static final double MAX_ANGLE = Math.toRadians(180);
        public static final double MIN_ANGLE = Math.toRadians(-135);


        public static final double TURRET_OFFSET = 0;


        public static  double KP = 2.0;
        public static  double KI = 0.0;
        public static  double KD = 0.08;

        public static final double MAX_POWER = 0.7;

        public static final double MANUAL_POWER = 0.5;
    }
    /* ========== LIMELIGHT ========== */
    public static class Limelight{
        public static String LIMELIGHT_NAME = "limelight";
        public static int LIMELIGHT_POLL_RATE_HZ = 100;
        public static int DEFAULT_PIPELINE = 0;

        public static double LIMELIGHT_HEIGHT_INCHES = 13.718;
        public static double LIMELIGHT_PITCH_DEGREES = 11.33187;

        public static double TARGET_HEIGHT_INCHES = 30.0;


        public static double DISTANCE_OFFSET_INCHES = 0.0;
        public static double RPM_OFFSET = 0.0;

        public static final double GRAVITY = 386.09;



    }


}
