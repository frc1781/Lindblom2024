
package tech.team1781;

import javax.print.DocFlavor.STRING;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import tech.team1781.utils.EVector;

public class ConfigMap {
        public static final ShuffleboardTab SHUFFLEBOARD_TAB = Shuffleboard.getTab("Electric Eagles");
        public static final ShuffleboardTab AUTONOMOUS_TAB = Shuffleboard.getTab("Autonomous");

        public static final ShuffleboardTab CONFIG_TAB = Shuffleboard.getTab("Config");

        public static final EVector RED_SPEAKER_LOCATION = EVector.newVector(16.77, 5.54);
        public static final EVector BLUE_SPEAKER_LOCATION = EVector.newVector(-0.2, 5.54);
        
        public static final EVector BLUE_PODIUM = EVector.newVector(3,4.7);
        public static final EVector RED_PODIUM = EVector.newVector(13.6,3.6);


        public static final double MAX_SHOOTER_SPEED = 7;

        // CAN IDs

        // 20-24 Drive train left
        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 21;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 20;
        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 23;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 22;

        // 25-29 Drive train right
        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 26;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 25;
        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 28;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 27;

        // 40-41 Arm

        public static final int ARM_PIVOT_LEFT_MOTOR = 40;
        public static final int ARM_PIVOT_RIGHT_MOTOR = 41;
        public static final double ARM_POSITION_TOLERANCE = 2.0;
        public static final double ARM_GEAR_RATIO = (1.0/125.0)*(18.0/56.0);

        // 57 Motors
        public static final int COLLECTOR_MOTOR = 57;
        public static final int SHOOTER_TOP_MOTOR = 42;
        public static final int SHOOTER_BOTTOM_MOTOR = 43;
        public static final int LEFT_CLIMBER_MOTOR = 50;
        public static final int RIGHT_CLIMBER_MOTOR = 51;

        // 60-69 Sensors
        public static final int BOTTOM_SCOLLECTOR_TOF = 58;
        public static final int TOP_SCOLLECTOR_TOF = 56;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 60;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 62;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 61;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 59;

        // Swerve

        public static final double DRIVETRAIN_TRACKWIDTH = Units.inchesToMeters(30);
        public static final double DRIVETRAIN_WHEELBASE = Units.inchesToMeters(28);
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.5;
        public static final double MAX_VELOCITY_FOR_UPDATE = 0.01;
        public static final double MAX_VELOCITY_RADIANS_PER_SECOND = (MAX_VELOCITY_METERS_PER_SECOND /
                        (Math.hypot(DRIVETRAIN_TRACKWIDTH / 2, DRIVETRAIN_WHEELBASE / 2)));
        public static Translation2d FRONT_LEFT_MODULE_POSITION = new Translation2d(DRIVETRAIN_WHEELBASE / 2,
                        DRIVETRAIN_TRACKWIDTH / 2);
        public static Translation2d FRONT_RIGHT_MODULE_POSITION = new Translation2d(DRIVETRAIN_WHEELBASE / 2,
                        -DRIVETRAIN_TRACKWIDTH / 2);
        public static Translation2d BACK_LEFT_MODULE_POSITION = new Translation2d(-DRIVETRAIN_WHEELBASE / 2,
                        DRIVETRAIN_TRACKWIDTH / 2);
        public static Translation2d BACK_RIGHT_MODULE_POSITION = new Translation2d(-DRIVETRAIN_WHEELBASE / 2,
                        -DRIVETRAIN_TRACKWIDTH / 2);

        // public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -0.0454; //
        // -0.919; // -0.075;
        // public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -0.0185;
        // //-0.738; // -0.255;
        // public static final double BACK_LEFT_MODULE_STEER_OFFSET = -0.8615; //-0.927;
        // // -0.077;
        // public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -0.8688;
        // //-0.272; // -0.735;

        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -0.9755;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -0.8167;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -0.670;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -0.116;

        // Constants
        public static final double MIN_THRESHOLD_ARM = 0;
        public static final double MAX_THRESHOLD_ARM = 90;

        // First Drivebase

        //Limelight
        public static final String FRONT_LIMELIGHT_NAME = "limelight-back";
        public static final String BACK_LIMELIGHT_NAME = "limelight-front";

        // Controls
        public static final int DRIVER_CONTROLLER_PORT = 0;

        public static final double DRIVER_TRANSLATION_RATE_LIMIT = 1.2;
        public static final double DRIVER_ROTATION_RATE_LIMIT = 1.2;

        public static final String RESET_NAVX = "X";
        public static final String CALIBRATE_POSITION = "Y";
        public static final String KEEP_DOWN = "LB";
        public static final String COLLECT = "RB";
        public static final String AUTO_AIM = "A";
        public static final String COLLECT_HIGH = "B";

        // Co-pilot controls
        public static final int CO_PILOT_PORT = 1;

        public static final String NOTE_COLLECTION = "B";
        public static final String SPIT = "LB";
        public static final String SHOOT = "RB";
        public static final String PREPARE_TO_SHOOT = "X";
        public static final String ANGLE_UP = "E";
        public static final String ANGLE_DOWN = "W";
        public static final String SCORE_AMP = "Y";
        // public static final String CLIMBER_EXTEND = "N";
        // public static final String CLIMB_RETRACT = "S";
}