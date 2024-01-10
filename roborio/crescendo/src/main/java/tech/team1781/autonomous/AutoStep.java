package tech.team1781.autonomous;

//TODO: Change to pathplanner trajectory
import edu.wpi.first.math.trajectory.Trajectory;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class AutoStep {
    private double mMaxTime = -1;
    private Action mAction = null;
    private EVector mPosition = null;
    private Trajectory mTrajectory = null;

    public AutoStep(double maxTime, Action action, EVector position) {
        mMaxTime = maxTime;
        mAction = action;
        mPosition = position;
    }

    public AutoStep(double maxTime, Action action, Trajectory trajectory) {
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

    public AutoStep(double maxTime, Trajectory trajectory) {
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

    public Trajectory getTrajectory() {
        return mTrajectory;
    }

}
