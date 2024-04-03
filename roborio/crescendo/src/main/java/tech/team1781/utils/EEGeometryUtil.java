package tech.team1781.utils;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class EEGeometryUtil {
    private static final double FIELD_LENGTH = 16.54;

    public static Rotation2d normalizeAngle(Rotation2d angle) {
       double radians = angle.getRadians();
       
       radians %= 2 * Math.PI;
       if(radians<0) {
           radians += 2 * Math.PI;
       }

        return Rotation2d.fromRadians(radians);
    }

    public static EVector flipPosition(EVector position) {
        EVector ret_val = new EVector(FIELD_LENGTH - position.x, position.y);
        ret_val.z = -position.z + Math.PI;
        ret_val.z = normalizeAngle(Rotation2d.fromRadians(ret_val.z)).getRadians();
        return ret_val;
    }
}
