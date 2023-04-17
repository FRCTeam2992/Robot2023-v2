package frc.lib.manipulator;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorState;

public final class Constants {
        public static final double MAX_HEIGHT_INCHES = ElevatorConstants.Limits.hardStopTop;
        public static final double MIN_HEIGHT_INCHES = ElevatorConstants.Limits.hardStopBottom;
        public static final double MIN_ANGLE_DEGREES = ArmConstants.Limits.hardStopBottom;
        public static final double MAX_ANGLE_DEGREES = ArmConstants.Limits.hardStopTop;
        public static final double NEUTRAL_DEGREES = 11.0;
        public static final double INSIDE_ZONE_INCHES = 7.0;
        public static final double INSIDE_ENTRY_DEGREES = 9.25;
        public static final double INSIDE_EXIT_DEGREES = -54.0;
        public static final double GROUND_HIT_INCHES = 11.5;
        public static final double GROUND_HIT_ANGLE = 75.0;

        public static final class NoFlyZones {
            // Corner x, corner y, width, height, zonetype
            public static final NoFlyZone HI_INSIDE = new NoFlyZone(INSIDE_ZONE_INCHES,
                    INSIDE_EXIT_DEGREES, MAX_HEIGHT_INCHES, INSIDE_ENTRY_DEGREES, NoFlyZone.ZoneType.Always);
            public static final NoFlyZone GROUND_HIT = new NoFlyZone(MIN_HEIGHT_INCHES,
                    GROUND_HIT_ANGLE, GROUND_HIT_INCHES, MAX_ANGLE_DEGREES,
                    NoFlyZone.ZoneType.ElevatorDeployed);
        }

        public static final class Boundaries {
                public static final NoFlyBoundary BOUNDARY_BASE = new NoFlyBoundary(
                                NoFlyBoundary.NoFlyBoundaryType.AT_ANGLE,
                                0.0, 6.0, 90.0);
                public static final NoFlyBoundary BOUNDARY_CROSSBAR = new NoFlyBoundary(
                                NoFlyBoundary.NoFlyBoundaryType.AT_HEIGHT,
                                MAX_ANGLE_DEGREES - 10.0, MAX_ANGLE_DEGREES, MAX_HEIGHT_INCHES - 3.0);
        }

        public static final class Waypoints {
            public static final Waypoint HI_OUTSIDE_EDGE = new Waypoint(INSIDE_ZONE_INCHES, INSIDE_ENTRY_DEGREES,
                    Waypoint.OuttakeType.Unknown, ElevatorState.Undeployed, 0.0);
            public static final Waypoint HI_INSIDE_EDGE = new Waypoint(INSIDE_ZONE_INCHES, INSIDE_EXIT_DEGREES,
                    Waypoint.OuttakeType.Unknown, ElevatorState.Undeployed, 0.0);
            public static final Waypoint GROUND_EXIT = new Waypoint(GROUND_HIT_INCHES, GROUND_HIT_ANGLE,
                    Waypoint.OuttakeType.Unknown, ElevatorState.Undeployed, 0.0);
            public static final Waypoint NEUTRAL = new Waypoint(0.0, NEUTRAL_DEGREES,
                    Waypoint.OuttakeType.Unknown, ElevatorState.Undeployed, 0.0);
        }
}
