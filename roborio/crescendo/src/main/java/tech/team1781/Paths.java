package tech.team1781;

import java.util.HashMap;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

public class Paths {

    private static final HashMap<AutonomousPosition, HashMap<AutonomousPosition, Junction>> junctions = new HashMap<>();

    static {
        junctions.put(AutonomousPosition.POSITION_1, new HashMap<>());
        junctions.put(AutonomousPosition.POSITION_2, new HashMap<>());
        junctions.put(AutonomousPosition.POSITION_3, new HashMap<>());
        junctions.put(AutonomousPosition.NOTE_1, new HashMap<>());
        junctions.put(AutonomousPosition.NOTE_2, new HashMap<>());
        junctions.put(AutonomousPosition.NOTE_3, new HashMap<>());
        junctions.put(AutonomousPosition.CENTER_1, new HashMap<>());
        junctions.put(AutonomousPosition.CENTER_2, new HashMap<>());
        junctions.put(AutonomousPosition.CENTER_3, new HashMap<>());
        junctions.put(AutonomousPosition.CENTER_4, new HashMap<>());
        junctions.put(AutonomousPosition.CENTER_5, new HashMap<>());


    }

    public enum AutonomousPosition {
        POSITION_1,
        POSITION_2,
        POSITION_3,
        NOTE_1,
        NOTE_2,
        NOTE_3,
        CENTER_1,
        CENTER_2,
        CENTER_3,
        CENTER_4,
        CENTER_5;

        

        // private HashMap<AutonomousPosition, Junction> junctions = new HashMap<>();

        // private AutonomousPosition(Junction... junctions) {
        //     for (Junction junction : junctions) {
        //         this.junctions.put(junction.endPosition, junction);
        //     }
        // }

        // public PathPlannerPath getPathTo(AutonomousPosition position) {
        //     return junctions.get(position).path;
        // }

    }

    private static void addJunction(AutonomousPosition start, AutonomousPosition end, PathPlannerPath path) {
        Junction junction = new Junction(end, path);
        junctions.get(start).put(end, junction);
    }

    public static class Junction {
        public AutonomousPosition endPosition;
        public PathPlannerPath path;

        public Junction(AutonomousPosition endPosition, PathPlannerPath path) {
            this.endPosition = endPosition;
            this.path = path;
        }
    }


}
