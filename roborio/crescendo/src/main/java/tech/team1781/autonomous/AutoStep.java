package tech.team1781.autonomous;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class AutoStep {
    private double mMaxTime = -1;
    private Action mAction = null;
    private EVector mPosition = null;
    private PathPlannerTrajectory mTrajectory = null;

    public AutoStep(double maxTime, Action action, EVector position) {
        mMaxTime = maxTime;
        mAction = action;
        mPosition = position;
    }

    public AutoStep(double maxTime, Action action, PathPlannerTrajectory trajectory) {
        mMaxTime = maxTime;
        mAction = action;
        mTrajectory = trajectory;
    }

    public AutoStep(double maxTime, Action action) {
        mMaxTime = maxTime;
        mAction = action;
    }

    public AutoStep(double maxTime, EVector position) {
        mMaxTime = maxTime;
        mPosition = position;
    }

    public AutoStep(double maxTime, PathPlannerTrajectory trajectory) {
        mMaxTime = maxTime;
        mTrajectory = trajectory;
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

    public PathPlannerTrajectory getTrajectory() {
        return mTrajectory;
    }

}
