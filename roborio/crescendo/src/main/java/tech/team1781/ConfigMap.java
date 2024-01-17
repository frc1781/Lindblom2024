
package tech.team1781;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ConfigMap {
    public static final ShuffleboardTab SHUFFLEBOARD_TAB = Shuffleboard.getTab("Electric Eagles");

    public static final String BEST_TEAM_MEMBER = "Vincent";

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

    // 60-69 Sensors
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 60;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 62;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 61;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 59;

    // Swerve

    public static final double DRIVETRAIN_TRACKWIDTH = Units.inchesToMeters(30);
    public static final double DRIVETRAIN_WHEELBASE = Units.inchesToMeters(25);
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.0;
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

    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -0.0454; // -0.919; // -0.075;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -0.0185; //-0.738; // -0.255;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -0.8615; //-0.927; // -0.077;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -0.8688; //-0.272; // -0.735;

    // First Drivebase

    // Controls
    public static final int DRIVER_CONTROLLER_PORT = 0;

    public static final double DRIVER_TRANSLATION_RATE_LIMIT = 1.2;
    public static final double DRIVER_ROTATION_RATE_LIMIT = 1.2;

    public static final String RESET_NAVX = "X";
}