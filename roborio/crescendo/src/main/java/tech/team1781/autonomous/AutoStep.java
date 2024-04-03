package tech.team1781.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class AutoStep {
    private double mMaxTime = -1;
    private Action mAction = null;
    private WaypointHolder mWaypointHolder = null;
    private PathPlannerPath mPath = null;
    private StepType mType = null;

    public AutoStep(double maxTime, Action action, EVector psEVector, double speedMetersPerSecond) {
        mMaxTime = maxTime;
        mAction = action;
        mWaypointHolder = new WaypointHolder(psEVector.x, psEVector.y, psEVector.z, speedMetersPerSecond);

        mType = StepType.POSITION_AND_ACTION;
    }

    public AutoStep(double maxTime, Action action, EVector psEVector, double speedMetersPerSecond, boolean isNotePosition) {
        mMaxTime = maxTime;
        mAction = action;
        mWaypointHolder = new WaypointHolder(psEVector.x, psEVector.y, psEVector.z, speedMetersPerSecond);

        if(isNotePosition)
            mType = StepType.NOTE_POSITION;
        else 
            mType = StepType.POSITION_AND_ACTION;
    }

    public AutoStep(double maxTime, Action action, PathPlannerPath path) {
        mMaxTime = maxTime;
        mAction = action;
        mPath = path;

        mType = StepType.PATH_AND_ACTION;
    }

    public AutoStep(double maxTime, Action action) {
        mMaxTime = maxTime;
        mAction = action;

        mType = StepType.ACTION;
    }

    public AutoStep(double maxTime, EVector position, double speedMetersPerSecond) {
        mMaxTime = maxTime;
        mWaypointHolder = new WaypointHolder(position.x, position.y, position.z, speedMetersPerSecond);

        mType = StepType.POSITION;
    }

    public AutoStep(double maxTime, PathPlannerPath path) {
        mMaxTime = maxTime;
        mPath = path;

        mType = StepType.PATH;
    }

    public AutoStep(double maxTime, double angleRads) {
        mWaypointHolder = new WaypointHolder(0, 0, angleRads, 0);
        mMaxTime = maxTime;
        
        mType = StepType.ROTATION;
    }

    public AutoStep(double maxTime, double angleRads, Action action) {
        mWaypointHolder = new WaypointHolder(0, 0, angleRads, 0);
        mMaxTime = maxTime;
        mAction = action;
        
        mType = StepType.ROTATION_AND_ACTION;
    }

    public AutoStep(double maxTime) {
        mMaxTime = maxTime;

        mType = StepType.WAIT;
    }

    public double getMaxTime() {
        return mMaxTime;
    }

    public Action getAction() {
        return mAction;
    }

    public WaypointHolder getWaypointHolder() {
        return mWaypointHolder;
    }

    public PathPlannerPath getPath() {
        return mPath;
    }

    public StepType getType() {
        return mType;
    }

    @Override
    public String toString() {
        switch (mType) {
            case ACTION:
                return "AutoStep: ( Time: " + mMaxTime + ", Action: " + mAction + ")";
            case POSITION:
                return "AutoStep: ( Time: " + mMaxTime + ", Position: " + mWaypointHolder.getPosition() + ", Speed MPS: "+ mWaypointHolder.getSpeedMetersPerSecond() + ")";
            case PATH:
                return "AutoStep: ( Time: " + mMaxTime + ", Path: " + mPath + ")";
            case WAIT:
                return "AutoStep: ( Time: " + mMaxTime + ")";
            case POSITION_AND_ACTION:
            case NOTE_POSITION:
                return "AutoStep: ( Time: " + mMaxTime + ", Action: " + mAction + ", Position: " + mWaypointHolder.getPosition() + ", Speed MPS: "+ mWaypointHolder.getSpeedMetersPerSecond() + ")";
            case PATH_AND_ACTION:
                return "AutoStep: ( Time: " + mMaxTime + ", Action: " + mAction + ", Path: " + mPath + ")";
            case ROTATION:
                return "AutoStep: ( Time: " + mMaxTime + ", Rotation: " + mWaypointHolder.getPosition().z + ")";
            case ROTATION_AND_ACTION:
                return "AutoStep: ( Time: " + mMaxTime + ", Action: " + mAction + ", Rotation: " + mWaypointHolder.getPosition().z + ")";
            default:
                return "AutoStep: ( Time: " + mMaxTime + ")";
        }
    }

    public enum StepType {
        ACTION,
        POSITION,
        ROTATION,
        NOTE_POSITION,
        PATH,
        WAIT,
        POSITION_AND_ACTION,
        ROTATION_AND_ACTION,
        PATH_AND_ACTION
    }

}
