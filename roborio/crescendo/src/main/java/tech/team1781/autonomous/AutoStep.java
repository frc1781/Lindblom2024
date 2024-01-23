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

    public AutoStep(double maxTime, Action action, EVector position) {
        mMaxTime = maxTime;
        mAction = action;
        mPosition = position;
    }

    public AutoStep(double maxTime, Action action, PathPlannerPath path) {
        mMaxTime = maxTime;
        mAction = action;
        mPath = path;
    }

    public AutoStep(double maxTime, Action action) {
        mMaxTime = maxTime;
        mAction = action;
    }

    public AutoStep(double maxTime, EVector position) {
        mMaxTime = maxTime;
        mPosition = position;
    }

    public AutoStep(double maxTime, PathPlannerPath path) {
        mMaxTime = maxTime;
        mPath = path;
    }

    public String toString() {
        return mAction.toString();
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

}
