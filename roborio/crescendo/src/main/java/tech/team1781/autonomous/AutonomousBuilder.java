package tech.team1781.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import org.littletonrobotics.junction.Logger;
import tech.team1781.Paths;
import tech.team1781.Paths.AutonomousPosition;
import tech.team1781.control.ControlSystem;
import tech.team1781.control.ControlSystem.Action;

import java.util.LinkedHashMap;
import java.util.LinkedList;

public class AutonomousBuilder {

    public static AutoStep[] buildFromLinkedHashMap(LinkedHashMap<GenericEntry, Paths.AutonomousPosition> map) {
        LinkedList<Paths.AutonomousPosition> positions = new LinkedList<>();

        for (GenericEntry entry : map.keySet()) {
            if (entry.getBoolean(false)) {
                positions.add(map.get(entry));
            }
        }

        Paths.AutonomousPosition[] positionsArray = new Paths.AutonomousPosition[positions.size()];

        for (int i = 0; i < positions.size(); i++) {
            positionsArray[i] = positions.get(i);
        }

        return buildFromPositions(positionsArray);
    }

    public static AutoStep[] buildFromString(String s) {
        s = s.trim();
        String[] steps = s.split(",");
        if (steps.length == 0) {
            throw new IllegalArgumentException("Invalid string format");
        }

        LinkedList<AutoStep> autonomousSteps = new LinkedList<>();
        String previous = steps[0];
        autonomousSteps.add(new AutoStep(2, ControlSystem.Action.SHOOT));

        for (int i = 1; i < steps.length; i++) {
            String current = steps[i];
            if (isWait(current)) {
                String timeInt = current.substring(1);
                int time = Integer.parseInt(timeInt);
                AutoStep waitStep = new AutoStep(time);
                autonomousSteps.add(waitStep);
                continue;

            }

            AutoStep currentStep;
            Paths.AutonomousPosition prevPosition = Paths.getPosition(previous);
            Paths.AutonomousPosition nextPosition = Paths.getPosition(current);

            PathPlannerPath path = Paths.getPath(prevPosition, nextPosition);
            System.out.println("Path: " + path.toString()
                    + " ############################################################################################################# ");

            PathPlannerTrajectory tempTraj = path.getTrajectory(new ChassisSpeeds(), new Rotation2d());
            Logger.recordOutput("Autonomus/currentPath", path.toString()  );
            final double shootWaitTimeAfterPath = 2.5;
            currentStep = new AutoStep(tempTraj.getTotalTimeSeconds() + shootWaitTimeAfterPath,
                    Action.COLLECT_AUTO_SHOOT, path);
            autonomousSteps.add(currentStep);
            previous = steps[i];

        }

        AutoStep[] ret_val = new AutoStep[autonomousSteps.size()];

        for (int i = 0; i < autonomousSteps.size(); i++) {
            ret_val[i] = autonomousSteps.get(i);
        }

        return ret_val;
    }

    public static AutoStep[] buildFromPositions(Paths.AutonomousPosition... positions) {
        // Paths paths = new Paths();

        // LinkedList<AutoStep> autonomousSteps = new LinkedList<>();
        // final double shootTime = 5;
        // final double rampedShootTime = 1;
        // autonomousSteps.add(new AutoStep(100, Action.OFF_KICKSTAND));
        // autonomousSteps.add(new AutoStep(100, Action.COLLECT_RAMP));
        // autonomousSteps.add(new AutoStep(shootTime, Action.COLLECT_AUTO_SHOOT));

        // Paths.AutonomousPosition previous = positions[0];
        // Paths.AutonomousPosition lastNonCenter = null;
        // for (int i = 1; i < positions.length; i++) {
        //     Paths.AutonomousPosition current = positions[i];
        //     if (isCenter(current)) {
        //         PathPlannerPath toCenterPath;
        //         PathPlannerPath fromCenterPath = Paths.getPathFromName("c;back");
        //         if (isCenter(previous)) {
        //             // TEMPORARY PATH
        //             toCenterPath = paths.getPathNonStatic(AutonomousPosition.NOTE_1, Paths.AutonomousPosition.CENTER_1);
        //         } else {
        //             toCenterPath = paths.getPathNonStatic(previous, current);
        //         }

        //         final double WAIT_TIME = 2;
        //         final double SEEK_TIME = 5; 
        //         final double POSITION_WAIT = 6;
        //         double toCenterTime = toCenterPath.getTrajectory(new ChassisSpeeds(), new Rotation2d()).getTotalTimeSeconds();
        //         double fromCenterTime = fromCenterPath.getTrajectory(new ChassisSpeeds(), new Rotation2d()).getTotalTimeSeconds();

        //         var toCenter = new AutoStep(toCenterTime + WAIT_TIME, toCenterPath);
        //         var seek = new AutoStep(SEEK_TIME, Action.SEEK_NOTE);
        //         var toStarting = new AutoStep(POSITION_WAIT, EVector.fromPose(toCenterPath.getPreviewStartingHolonomicPose()));
        //         var toBack = new AutoStep(fromCenterTime + WAIT_TIME, Action.COLLECT_RAMP,fromCenterPath);

        //         autonomousSteps.add(toCenter);
        //         autonomousSteps.add(seek);
        //         autonomousSteps.add(toStarting);
        //         autonomousSteps.add(toBack);

        //     } else {
        //         lastNonCenter = current;
        //         PathPlannerPath path = paths.getPathNonStatic(previous, current);

        //         PathPlannerTrajectory tempTraj = path.getTrajectory(new ChassisSpeeds(), new Rotation2d());
        //         final double shootWaitTimeAfterPath = 0;
        //         AutoStep currentStep = new AutoStep(tempTraj.getTotalTimeSeconds() + shootWaitTimeAfterPath,
        //                 Action.COLLECT_RAMP, path);
        //         autonomousSteps.add(currentStep);
        //     }

        //     AutoStep shootStep = new AutoStep(rampedShootTime, Action.AUTO_AIM_SHOOT);
        //     autonomousSteps.add(shootStep);

            
        //     previous = positions[i];
        // }

        // AutoStep[] ret_val = new AutoStep[autonomousSteps.size()];
        // System.out.println("length: " + autonomousSteps.size());

        // for (int i = 0; i < autonomousSteps.size(); i++) {
        //     ret_val[i] = autonomousSteps.get(i);
        //     System.out.println(ret_val[i].toString());
        // }

        // return ret_val;
        return null;

    }

    public static boolean isCenter(AutonomousPosition position) {
        return position == Paths.AutonomousPosition.CENTER_1 ||
                position == Paths.AutonomousPosition.CENTER_2 ||
                position == Paths.AutonomousPosition.CENTER_3 ||
                position == Paths.AutonomousPosition.CENTER_4 ||
                position == Paths.AutonomousPosition.CENTER_5;
    }

    private static boolean isWait(String s) {
        return s.startsWith("w");
    }
}
