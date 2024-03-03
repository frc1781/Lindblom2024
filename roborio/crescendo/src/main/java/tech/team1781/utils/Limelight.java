package tech.team1781.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.fasterxml.jackson.annotation.JsonFormat.Shape;
import com.fasterxml.jackson.annotation.JsonProperty;
import tech.team1781.ConfigMap;

public class Limelight {

    public static Pose2d getBotPose2d(String limelightName) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);
        double[] values = table.getEntry("botpose").getDoubleArray(new double[6]);

        return new Pose2d(values[0], values[1], new Rotation2d(values[5]));
    }

    public static double getTX(String limelightName) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);
        return table.getEntry("tx").getDouble(0.0);
    }

    public static double getTY(String limelightName) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);
        return table.getEntry("ty").getDouble(0.0);
    }

    public static double getTA(String limelightName) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);
        return table.getEntry("ta").getDouble(0.0);
    }

    public static String getLatestRawJSON(String limelightName) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);
        return table.getEntry("json").getString("{}");
    }

    public static int getTV(String limelightName) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);
        return (int) table.getEntry("tv").getDouble(0.0);
    }

    public static double getNumberOfApriltags(String limelightName) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);
        double[] values = table.getEntry("botpose").getDoubleArray(new double[0]);

        if(values.length < 8) {
            return 0;
        }

        return values[7];
    }

    public static void setTargetApriltag(String limelightName, int id) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);
        table.getEntry("priorityid").setDouble(id);
    }
}