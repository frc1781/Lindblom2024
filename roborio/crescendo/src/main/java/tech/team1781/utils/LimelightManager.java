package tech.team1781.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightManager {

    private static LimelightManager instance = null;

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry xCoord = table.getEntry("tx");
    private NetworkTableEntry yCoord = table.getEntry("ty");
    private NetworkTableEntry rotationDegrees = table.getEntry("rz");

    public LimelightManager getInstance() {
        if (instance == null) {
            instance = new LimelightManager();
        }

        return  instance;
    }

    public Pose2d getPose() {
        double x = xCoord.getDouble(0.0);
        double y = yCoord.getDouble(0.0);
        double rotation = rotationDegrees.getDouble(0.0);

        return new Pose2d(x, y, new Rotation2d(rotation));
    }
}
