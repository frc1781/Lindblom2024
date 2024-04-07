package tech.team1781.autonomous;

import tech.team1781.utils.EVector;

public class WaypointHolder {
    private EVector mPosition;
    private double mSpeedMetersPerSecond;
    
    public WaypointHolder(double x, double y, double rotation, double speedMetersPerSecond) {
        mPosition = EVector.newVector(x, y, rotation);
        mSpeedMetersPerSecond = speedMetersPerSecond;
    }

    public WaypointHolder(EVector position, double speedMetersPerSecond) {
        mPosition = position;
        mSpeedMetersPerSecond = speedMetersPerSecond;
    }

    public double getSpeedMetersPerSecond() {
        return mSpeedMetersPerSecond;
    }

    public EVector getPosition() {
        return mPosition;
    }

    public WaypointHolder changeX(double x) {
        mPosition = mPosition.withX(x);
        return this;
    }

    public WaypointHolder changeY(double y) {
        mPosition = mPosition.withY(y);
        return this;
    }

    public WaypointHolder changeRotation(double rotation) {
        mPosition = mPosition.withZ(rotation);
        return this;
    }

    public WaypointHolder changeSpeed(double speedMetersPerSecond) {
        mSpeedMetersPerSecond = speedMetersPerSecond;
        return this;
    }

    public WaypointHolder copy() {
        var newPosition = mPosition.flipIfRed();
        return new WaypointHolder(newPosition.x, newPosition.y, newPosition.z, mSpeedMetersPerSecond);
    }


}
