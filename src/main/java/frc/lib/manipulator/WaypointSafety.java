package frc.lib.manipulator;

public class WaypointSafety {
    static public enum WaypointSafetyClassification {
        Safe,
        Inside_TooHigh,
        Ground_If_Deployed
    }

    static public WaypointSafetyClassification nonSafeZones(Waypoint p) {
        // System.out.println(">>>>>>>>>>>>>>>>> WayPointSafety: testing point: {" +
        // p.height() + "," + p.angle() + ")");
        if (Constants.NoFlyZones.HI_INSIDE.contains(p)) {
            // System.out.println("Returning Inside_TooHigh");
            return WaypointSafetyClassification.Inside_TooHigh;
        }
        if (Constants.NoFlyZones.GROUND_HIT.contains(p)) {
            // System.out.println("Returning Ground_If_Deployed");
            return WaypointSafetyClassification.Ground_If_Deployed;
        }
        // System.out.println("Returning Safe");
        return WaypointSafetyClassification.Safe;
    }
}
