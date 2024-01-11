package tech.team1781;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ConfigMap {
    public static final ShuffleboardTab SHUFFLEBOARD_TAB = Shuffleboard.getTab("Electric Eagles");

    public static final String BEST_TEAM_MEMBER = "Vincent";    
    
    //Swerve
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.0;

    public static Translation2d FRONT_LEFT_MODULE_POSITION = new Translation2d();
    public static Translation2d FRONT_RIGHT_MODULE_POSITION = new Translation2d();
    public static Translation2d BACK_LEFT_MODULE_POSITION = new Translation2d();
    public static Translation2d BACK_RIGHT_MODULE_POSITION = new Translation2d();

    //First Drivebase 
}