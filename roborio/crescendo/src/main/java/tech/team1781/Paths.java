package tech.team1781;

import java.util.HashMap;

import com.pathplanner.lib.path.PathPlannerPath;

import tech.team1781.autonomous.AutonomousBuilder;
import tech.team1781.control.ControlSystem;

public class Paths {

    private static final PathPlannerPath position1ToNote1 = PathPlannerPath.fromPathFile("p1;n1");
    private static final PathPlannerPath position1ToNote2 = PathPlannerPath.fromPathFile("p1;n2");
    private static final PathPlannerPath position1ToNote3 = PathPlannerPath.fromPathFile("p1;n3");
    private static final PathPlannerPath position1ToCenter1 = PathPlannerPath.fromPathFile("p1;c1");
    private static final PathPlannerPath position1ToCenter2 = PathPlannerPath.fromPathFile("p1;c2");
    private static final PathPlannerPath position1ToCenter3 = PathPlannerPath.fromPathFile("p1;c3");
    private static final PathPlannerPath position1ToCenter4 = PathPlannerPath.fromPathFile("p1;c4");
    private static final PathPlannerPath position1ToCenter5 = PathPlannerPath.fromPathFile("p1;c5");

    private static final PathPlannerPath position2ToNote1 = PathPlannerPath.fromPathFile("p2;n1");
    private static final PathPlannerPath position2ToNote2 = PathPlannerPath.fromPathFile("p2;n2");
    private static final PathPlannerPath position2ToNote3 = PathPlannerPath.fromPathFile("p2;n3");
    private static final PathPlannerPath position2ToCenter1 = PathPlannerPath.fromPathFile("p2;c1");
    private static final PathPlannerPath position2ToCenter2 = PathPlannerPath.fromPathFile("p2;c2");
    private static final PathPlannerPath position2ToCenter3 = PathPlannerPath.fromPathFile("p2;c3");
    private static final PathPlannerPath position2ToCenter4 = PathPlannerPath.fromPathFile("p2;c4");
    private static final PathPlannerPath position2ToCenter5 = PathPlannerPath.fromPathFile("p2;c5");

    private static final PathPlannerPath position3ToNote1 = PathPlannerPath.fromPathFile("p3;n1");
    private static final PathPlannerPath position3ToNote2 = PathPlannerPath.fromPathFile("p3;n2");
    private static final PathPlannerPath position3ToNote3 = PathPlannerPath.fromPathFile("p3;n3");
    private static final PathPlannerPath position3ToCenter1 = PathPlannerPath.fromPathFile("p3;c1");
    private static final PathPlannerPath position3ToCenter2 = PathPlannerPath.fromPathFile("p3;c2");
    private static final PathPlannerPath position3ToCenter3 = PathPlannerPath.fromPathFile("p3;c3");
    private static final PathPlannerPath position3ToCenter4 = PathPlannerPath.fromPathFile("p3;c4");
    private static final PathPlannerPath position3ToCenter5 = PathPlannerPath.fromPathFile("p3;c5");

    private static final PathPlannerPath note1ToNote2 = PathPlannerPath.fromPathFile("n1;n2");
    private static final PathPlannerPath note1ToNote3 = PathPlannerPath.fromPathFile("n1;n3");
    private static final PathPlannerPath note1ToCenter1 = PathPlannerPath.fromPathFile("n1;c2");
    private static final PathPlannerPath note1ToCenter2 = PathPlannerPath.fromPathFile("n1;c2");
    private static final PathPlannerPath note1ToCenter3 = PathPlannerPath.fromPathFile("n1;c3");
    private static final PathPlannerPath note1ToCenter4 = PathPlannerPath.fromPathFile("n1;c4");
    private static final PathPlannerPath note1ToCenter5 = PathPlannerPath.fromPathFile("n1;c5");

    private static final PathPlannerPath note2ToNote1 = PathPlannerPath.fromPathFile("n2;n1");
    private static final PathPlannerPath note2ToNote3 = PathPlannerPath.fromPathFile("n2;n3");
    private static final PathPlannerPath note2ToCenter1 = PathPlannerPath.fromPathFile("n2;c1");
    private static final PathPlannerPath note2ToCenter2 = PathPlannerPath.fromPathFile("n2;c2");
    private static final PathPlannerPath note2ToCenter3 = PathPlannerPath.fromPathFile("n2;c3");
    private static final PathPlannerPath note2ToCenter4 = PathPlannerPath.fromPathFile("n2;c4");
    private static final PathPlannerPath note2ToCenter5 = PathPlannerPath.fromPathFile("n2;c5");

    private static final PathPlannerPath note3ToNote1 = PathPlannerPath.fromPathFile("n3;n1");
    private static final PathPlannerPath note3ToNote2 = PathPlannerPath.fromPathFile("n3;n2");
    private static final PathPlannerPath note3ToCenter1 = PathPlannerPath.fromPathFile("n3;c1");
    private static final PathPlannerPath note3ToCenter2 = PathPlannerPath.fromPathFile("n3;c2");
    private static final PathPlannerPath note3ToCenter3 = PathPlannerPath.fromPathFile("n3;c3");
    private static final PathPlannerPath note3ToCenter4 = PathPlannerPath.fromPathFile("n3;c4");
    private static final PathPlannerPath note3ToCenter5 = PathPlannerPath.fromPathFile("n3;c5");

    private static final HashMap<AutonomousPosition, HashMap<AutonomousPosition, Junction>> junctions = new HashMap<>();
    private static final HashMap<String, AutonomousPosition> positionMap = new HashMap<>();

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

        addJunction(AutonomousPosition.POSITION_1, AutonomousPosition.NOTE_1, position1ToNote1);
        addJunction(AutonomousPosition.POSITION_1, AutonomousPosition.NOTE_2, position1ToNote2);
        addJunction(AutonomousPosition.POSITION_1, AutonomousPosition.NOTE_3, position1ToNote3);
        addJunction(AutonomousPosition.POSITION_1, AutonomousPosition.CENTER_1, position1ToCenter1);
        addJunction(AutonomousPosition.POSITION_1, AutonomousPosition.CENTER_2, position1ToCenter2);
        addJunction(AutonomousPosition.POSITION_1, AutonomousPosition.CENTER_3, position1ToCenter3);
        addJunction(AutonomousPosition.POSITION_1, AutonomousPosition.CENTER_4, position1ToCenter4);
        addJunction(AutonomousPosition.POSITION_1, AutonomousPosition.CENTER_5, position1ToCenter5);

        addJunction(AutonomousPosition.POSITION_2, AutonomousPosition.NOTE_1, position2ToNote1);
        addJunction(AutonomousPosition.POSITION_2, AutonomousPosition.NOTE_2, position2ToNote2);
        addJunction(AutonomousPosition.POSITION_2, AutonomousPosition.NOTE_3, position2ToNote3);
        addJunction(AutonomousPosition.POSITION_2, AutonomousPosition.CENTER_1, position2ToCenter1);
        addJunction(AutonomousPosition.POSITION_2, AutonomousPosition.CENTER_2, position2ToCenter2);
        addJunction(AutonomousPosition.POSITION_2, AutonomousPosition.CENTER_3, position2ToCenter3);
        addJunction(AutonomousPosition.POSITION_2, AutonomousPosition.CENTER_4, position2ToCenter4);
        addJunction(AutonomousPosition.POSITION_2, AutonomousPosition.CENTER_5, position2ToCenter5);

        addJunction(AutonomousPosition.POSITION_3, AutonomousPosition.NOTE_1, position3ToNote1);
        addJunction(AutonomousPosition.POSITION_3, AutonomousPosition.NOTE_2, position3ToNote2);
        addJunction(AutonomousPosition.POSITION_3, AutonomousPosition.NOTE_3, position3ToNote3);
        addJunction(AutonomousPosition.POSITION_3, AutonomousPosition.CENTER_1, position3ToCenter1);
        addJunction(AutonomousPosition.POSITION_3, AutonomousPosition.CENTER_2, position3ToCenter2);
        addJunction(AutonomousPosition.POSITION_3, AutonomousPosition.CENTER_3, position3ToCenter3);
        addJunction(AutonomousPosition.POSITION_3, AutonomousPosition.CENTER_4, position3ToCenter4);
        addJunction(AutonomousPosition.POSITION_3, AutonomousPosition.CENTER_5, position3ToCenter5);

        addJunction(AutonomousPosition.NOTE_1, AutonomousPosition.NOTE_2, note1ToNote2);
        addJunction(AutonomousPosition.NOTE_1, AutonomousPosition.NOTE_3, note1ToNote3);
        addJunction(AutonomousPosition.NOTE_1, AutonomousPosition.CENTER_1, note1ToCenter1);
        addJunction(AutonomousPosition.NOTE_1, AutonomousPosition.CENTER_2, note1ToCenter2);
        addJunction(AutonomousPosition.NOTE_1, AutonomousPosition.CENTER_3, note1ToCenter3);
        addJunction(AutonomousPosition.NOTE_1, AutonomousPosition.CENTER_4, note1ToCenter4);
        addJunction(AutonomousPosition.NOTE_1, AutonomousPosition.CENTER_5, note1ToCenter5);

        addJunction(AutonomousPosition.NOTE_2, AutonomousPosition.NOTE_1, note2ToNote1);
        addJunction(AutonomousPosition.NOTE_2, AutonomousPosition.NOTE_3, note2ToNote3);
        addJunction(AutonomousPosition.NOTE_2, AutonomousPosition.CENTER_1, note2ToCenter1);
        addJunction(AutonomousPosition.NOTE_2, AutonomousPosition.CENTER_2, note2ToCenter2);
        addJunction(AutonomousPosition.NOTE_2, AutonomousPosition.CENTER_3, note2ToCenter3);
        addJunction(AutonomousPosition.NOTE_2, AutonomousPosition.CENTER_4, note2ToCenter4);
        addJunction(AutonomousPosition.NOTE_2, AutonomousPosition.CENTER_5, note2ToCenter5);

        addJunction(AutonomousPosition.NOTE_3, AutonomousPosition.NOTE_1, note3ToNote1);
        addJunction(AutonomousPosition.NOTE_3, AutonomousPosition.NOTE_2, note3ToNote2);
        addJunction(AutonomousPosition.NOTE_3, AutonomousPosition.CENTER_1, note3ToCenter1);
        addJunction(AutonomousPosition.NOTE_3, AutonomousPosition.CENTER_2, note3ToCenter2);
        addJunction(AutonomousPosition.NOTE_3, AutonomousPosition.CENTER_3, note3ToCenter3);
        addJunction(AutonomousPosition.NOTE_3, AutonomousPosition.CENTER_4, note3ToCenter4);
        addJunction(AutonomousPosition.NOTE_3, AutonomousPosition.CENTER_5, note3ToCenter5);

        positionMap.put("p1", AutonomousPosition.POSITION_1);
        positionMap.put("p2", AutonomousPosition.POSITION_2);
        positionMap.put("p3", AutonomousPosition.POSITION_3);
        positionMap.put("n1", AutonomousPosition.NOTE_1);
        positionMap.put("n2", AutonomousPosition.NOTE_2);
        positionMap.put("n3", AutonomousPosition.NOTE_3);
        positionMap.put("c1", AutonomousPosition.CENTER_1);
        positionMap.put("c2", AutonomousPosition.CENTER_2);
        positionMap.put("c3", AutonomousPosition.CENTER_3);
        positionMap.put("c4", AutonomousPosition.CENTER_4);
        positionMap.put("c5", AutonomousPosition.CENTER_5);
    }

    public enum AutonomousPosition {
        POSITION_1(1, 1, "p1"),
        POSITION_2(2, 1, "p2"),
        POSITION_3(3, 1, "p3"),
        POSITION_1_RED(4,1, "rp1"),
        POSITION_3_RED(5,1, "rp3"),
        NOTE_1(1, 2, "n1"),
        NOTE_2(2, 2, "n2"),
        NOTE_3(3, 2, "n3"),
        CENTER_1(1, 3, "c1"),
        CENTER_2(2, 3, "c2"),
        CENTER_3(3, 3, "c3"),
        CENTER_4(4, 3, "c4"),
        CENTER_5(5, 3, "c5");


        private int xpos;
        private int ypos;
        private String name;

        private AutonomousPosition(int x, int y, String _name) {
            xpos = x;
            ypos = y;
            name = _name;
        }

        public String getName() {
            return name;
        }

        public int getX() {
            return xpos;
        }

        public int getY() {
            return ypos;
        }
    }

    public static class Junction {
        public AutonomousPosition endPosition;
        public PathPlannerPath path;

        public Junction(AutonomousPosition endPosition, PathPlannerPath path) {
            this.endPosition = endPosition;
            this.path = path;
        }
    }

    public PathPlannerPath getPathNonStatic(AutonomousPosition start, AutonomousPosition end) {
        PathPlannerPath ret_val;
        if(AutonomousBuilder.isCenter(end)) {
            ret_val = PathPlannerPath.fromPathFile(start.getName() + ";" + "c");    
        } else { 
            ret_val = PathPlannerPath.fromPathFile(concatenatePositions(start, end));
        }
        ret_val.preventFlipping = false;
        if(ControlSystem.isRed() && !start.getName().contains("r")) {
            ret_val = ret_val.flipPath();
        }
        return ret_val;
    }

    public static PathPlannerPath getPathFromName(String name) {
        var ret_val = PathPlannerPath.fromPathFile(name);
        ret_val.preventFlipping = false;
        if(ControlSystem.isRed()) {
            ret_val = ret_val.flipPath();
        }
        return ret_val;
    }


    public static AutonomousPosition getPosition(String position) {
        return positionMap.get(position);
    }

    public static PathPlannerPath getPath(AutonomousPosition start, AutonomousPosition end) {
        return junctions.get(start).get(end).path;
    }

    private String concatenatePositions(AutonomousPosition start, AutonomousPosition end) {
        return start.getName() + ";" + end.getName();
    }

    private static void addJunction(AutonomousPosition start, AutonomousPosition end, PathPlannerPath path) {
        Junction junction = new Junction(end, path);
        junctions.get(start).put(end, junction);
    }

}
