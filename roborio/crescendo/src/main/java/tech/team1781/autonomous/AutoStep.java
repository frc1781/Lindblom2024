package tech.team1781.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class AutoStep {
    private double mMaxTime = -1;
    private Action mAction = null;
    private EVector mPosition = null;
    private PathPlannerPath mPath = null;
    private StepType mType = null; 

    public AutoStep(double maxTime, Action action, EVector position) {
        mMaxTime = maxTime;
        mAction = action;
        mPosition = position;

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

    public AutoStep(double maxTime, EVector position) {
        mMaxTime = maxTime;
        mPosition = position;

        mType = StepType.POSITION;
    }

    public AutoStep(double maxTime, PathPlannerPath path) {
        mMaxTime = maxTime;
        mPath = path;

        mType = StepType.PATH;
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

    public EVector getPosition() {
        return mPosition;
    }

    public PathPlannerPath getPath() {
        return mPath;
    }

    public StepType getType() {
        return mType;
    }
    
    @Override 
    public String toString() {
        if(mPath == null) {
            return "AutoStep: ( Time: " + mMaxTime + ", Action: " + mAction + ", Position " + mPosition;
        }
        return "AutoStep: ( Time: " + mMaxTime + ", Action: " + mAction + ", Position " + mPosition + ", Path Starting Pose: " + mPath.getPreviewStartingHolonomicPose();
    }

    public enum StepType {
        ACTION,
        POSITION,
        PATH,
        WAIT,
        POSITION_AND_ACTION,
        PATH_AND_ACTION
    }

}
