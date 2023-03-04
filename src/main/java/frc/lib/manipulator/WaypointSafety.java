package frc.lib.manipulator;

public class WaypointSafety {
    static public enum WaypointSafetyClassification {
        Safe,
        NoFlyRobotBase,
        NoFlyCrossbar,
        PlannedPathRobotBase,
        PlannedPathCrossbar
    }

    static public WaypointSafetyClassification nonSafeZones(Waypoint p) {
        if (Constants.NoFlyZones.INSIDE_ROBOT.contains(p)) {
            return WaypointSafetyClassification.NoFlyRobotBase;
        }
        if (Constants.NoFlyZones.ELEVATOR_CROSSBAR.contains(p)) {
            return WaypointSafetyClassification.NoFlyCrossbar;
        }
        if (Constants.PlannedPathZones.ROBOT_BASE.contains(p)) {
            return WaypointSafetyClassification.PlannedPathRobotBase;
        }
        if (Constants.PlannedPathZones.ABOVE_CROSSBAR.contains(p)) {
            return WaypointSafetyClassification.PlannedPathCrossbar;
        }
        return WaypointSafetyClassification.Safe;
    }
}
