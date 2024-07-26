package tech.team1781;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import tech.team1781.utils.EVector;

public class ShuffleboardStyle {

    //Scollector
    public static final Style READY_TO_SHOOT = new Style(4, 4, 0, 0, BuiltInWidgets.kBooleanBox)
        .withProperty("Color when false", "#800000") //Maroon
        .withProperty("Color when true", "#03fc13"); //Green
    public static final Style HAS_NOTE = new Style(1, 1, 4, 1, BuiltInWidgets.kBooleanBox)
        .withProperty("Color when false", "#800000") //Maroon
        .withProperty("Color when true", "#ffbf00"); //Gold
    public static final Style SEES_NOTE = new Style(1,1,5,3, BuiltInWidgets.kBooleanBox)
        .withProperty("Color when false", "#800000") //Maroon
        .withProperty("Color when true", "#ffbf00"); //Gold
            
    public static final Style TOP_SHOOTER = new Style(1,1,4,2, BuiltInWidgets.kNumberBar)
        .withProperty("Min", 0)
        .withProperty("Max", ConfigMap.MAX_SHOOTER_SPEED);
    public static final Style BOTTOM_SHOOTER = new Style(1,1,4,3, BuiltInWidgets.kNumberBar)
        .withProperty("Min", 0)
        .withProperty("Max", ConfigMap.MAX_SHOOTER_SPEED);

    //Arm
    public static final Style ARM_ANGLE = new Style(1,1,4,0, BuiltInWidgets.kNumberBar)
        .withProperty("Min", 0)
        .withProperty("Max", 60);
    public static final Style ARM_AIM_SPOT = new Style(1,1, 8,3, BuiltInWidgets.kTextView);
        public static final Style ARM_ERROR = new Style(1,1,5,3, BuiltInWidgets.kBooleanBox)
        .withProperty("Color when false", "#ff0000") //Maroon
        .withProperty("Color when true", "#000000"); //Gold

    //DriveSystem
    public static final Style SEES_APRILTAG = new Style(1,1,5,3, BuiltInWidgets.kBooleanBox)
        .withProperty("Color when false", "#800000") //Maroon
        .withProperty("Color when true", "#ffbf00"); //Gold
    public static final Style ROBOT_X = new Style(1,1,5,0, BuiltInWidgets.kTextView);
    public static final Style ROBOT_Y = new Style(1,1,6,0, BuiltInWidgets.kTextView);
    public static final Style ROBOT_THETA = new Style(1,1,7,0, BuiltInWidgets.kTextView);
    public static final Style ROBOT_POSITION_FIELD = new Style(3, 2, 5, 1, BuiltInWidgets.kField);
    public static final Style ROBOT_X_VELOCITY = new Style(1,1,6,3, BuiltInWidgets.kNumberBar)
        .withProperty("Max", ConfigMap.MAX_VELOCITY_METERS_PER_SECOND)
        .withProperty("Min", 0);
    public static final Style ROBOT_Y_VELOCITY = new Style(1,1,7,3, BuiltInWidgets.kNumberBar)
        .withProperty("Max", ConfigMap.MAX_VELOCITY_METERS_PER_SECOND)
        .withProperty("Min", 0);
    public static final Style ROBOT_VELOCITY = new Style(1,1,8,0, BuiltInWidgets.kDial)
        .withProperty("Max", ConfigMap.MAX_VELOCITY_METERS_PER_SECOND)
        .withProperty("Min", 0);


    

    public static class Style {
        public EVector size;
        public EVector position;
        public BuiltInWidgets widgetType;
        public Map<String, Object> properties;

        public Style(int xsize, int ysize, int xpos, int ypos, BuiltInWidgets _widgetType){
            size = EVector.newVector(xsize, ysize);
            position = EVector.newVector(xpos, ypos);
            widgetType = _widgetType;
            properties = new HashMap<String, Object>();
        }

        public Style withProperty(String key, Object value) {
            properties.put(key, value);
            return this;
        }
        
    }

    public static GenericEntry getEntry(ShuffleboardTab tab, String name, Object defaultValue, Style style) {
        return tab.add(name, defaultValue)
            .withSize((int)style.size.x,(int) style.size.y)
            .withPosition((int) style.position.x, (int) style.position.y)
            .withWidget(style.widgetType)
            .withProperties(style.properties).getEntry();
            
    }
}
