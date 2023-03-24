package frc.robot.commands.groups;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.manipulator.Constants;
import frc.lib.manipulator.Waypoint;
import frc.lib.manipulator.WaypointSafety;
import frc.lib.manipulator.WaypointSafety.WaypointSafetyClassification;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;

public class SafeDumbTowerToPosition extends SequentialCommandGroup {
    Elevator mElevator;
    Arm mArm;
    Waypoint mEnd;

    public SafeDumbTowerToPosition(Elevator elevator, Arm arm, Waypoint point) {
        mElevator = elevator;
        mArm = arm;
        mEnd = point;
        addCommands(

                // First we check is we are starting in no go zone and move safely out if needed
                new SelectCommand(
                        Map.ofEntries(
                                Map.entry(
                                        WaypointSafety.WaypointSafetyClassification.Safe,
                                        new InstantCommand()),
                                Map.entry(
                                        WaypointSafety.WaypointSafetyClassification.Inside_TooHigh,
                                        new UnsafeMoveTowerToPosition(mElevator,
                                                mArm,
                                                frc.lib.manipulator.Constants.Waypoints.INSIDE_ENTRY)),
                                Map.entry(
                                        WaypointSafety.WaypointSafetyClassification.Ground_If_Deployed,
                                        new UnsafeMoveTowerToPosition(elevator,
                                                mArm,
                                                frc.lib.manipulator.Constants.Waypoints.GROUND_EXIT))),
                        this::checkStart),

                // Next we check if we're moving across the middle position with the arm,
                // and if so, we move to our safe center waypoint
                new SelectCommand(Map.ofEntries(
                        Map.entry(true, new UnsafeMoveTowerToPosition(
                                mElevator,
                                mArm,
                                frc.lib.manipulator.Constants.Waypoints.INSIDE_ENTRY)),
                        Map.entry(false, new InstantCommand())),
                        this::checkNeedsMiddleHeightAvoidWaypoint),

                // And now we can move into the final spot if it is safe
                new SelectCommand(
                        Map.ofEntries(
                                Map.entry(
                                        WaypointSafety.WaypointSafetyClassification.Safe,
                                        new UnsafeMoveTowerToPosition(mElevator,
                                                mArm, mEnd)),
                                Map.entry(
                                        WaypointSafety.WaypointSafetyClassification.Inside_TooHigh,
                                        new InstantCommand()),
                                Map.entry(
                                        WaypointSafety.WaypointSafetyClassification.Ground_If_Deployed,
                                        new InstantCommand())),
                        this::checkEnd));
    }

    private WaypointSafety.WaypointSafetyClassification checkStart() {
        WaypointSafety.WaypointSafetyClassification zoneClass = WaypointSafety.nonSafeZones(
                new Waypoint(mElevator.getElevatorInches(), mArm.getArmMotorPositionDeg()));
        if ((zoneClass == WaypointSafety.WaypointSafetyClassification.Ground_If_Deployed) &&
                (mElevator.getElevatorState() == ElevatorState.Deployed)) {
            return WaypointSafetyClassification.Ground_If_Deployed;
        } else {
            zoneClass = WaypointSafetyClassification.Safe;
        }
        return zoneClass;
    }

    private WaypointSafety.WaypointSafetyClassification checkEnd() {
        WaypointSafety.WaypointSafetyClassification zoneClass = WaypointSafety.nonSafeZones(mEnd);
        if ((zoneClass == WaypointSafetyClassification.Ground_If_Deployed) &&
                (mElevator.getElevatorState() == ElevatorState.Deployed)) {
            return WaypointSafetyClassification.Ground_If_Deployed;
        } else {
            zoneClass = WaypointSafetyClassification.Safe;
        }
        return zoneClass;
    }

    private boolean checkNeedsMiddleHeightAvoidWaypoint() {
        if (mArm.getArmMotorPositionDeg() > 7.0 && mEnd.angle() > 7.0) {
            return false;
        }
        if (mArm.getArmMotorPositionDeg() < -20.0 && mEnd.angle() < -20.0) {
            return false;
        }
        if ((mElevator.getElevatorInches() <= Constants.INSIDE_ENTRY_INCHES) &&
                (mEnd.height() <= Constants.INSIDE_ENTRY_INCHES)) {
            return false;
        }
        return true;
    }

}
