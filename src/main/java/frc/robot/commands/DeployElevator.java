// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.manipulator.Constants.Waypoints;
import frc.lib.manipulator.Waypoint;
import frc.lib.manipulator.WaypointSafety;
import frc.robot.RobotState;
import frc.robot.commands.groups.SafeDumbTowerToPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;

public class DeployElevator extends CommandBase {
    /** Creates a new DeployIntake. */
    private Elevator mElevator;
    private Arm mArm;
    private RobotState mRobotState;

    private ElevatorState mElevatorState;

    public DeployElevator(Elevator elevator, Arm arm, RobotState robotState, ElevatorState elevatorState) {
        // Use addRequirements() here to declare subsystem dependencies.
        mElevator = elevator;
        mArm = arm;
        mRobotState = robotState;

        mElevatorState = elevatorState;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // if (mElevatorState == ElevatorState.Deployed && mRobotState.towerIsMoving) {
        // WaypointSafety.WaypointSafetyClassification zoneClass =
        // WaypointSafety.nonSafeZones(
        // new Waypoint(
        // mRobotState.towerCurrentMoveTarget.height(),
        // mRobotState.towerCurrentMoveTarget.angle(),
        // Waypoint.OuttakeType.Unknown, ElevatorState.Deployed, 0.0));
        // if (zoneClass ==
        // WaypointSafety.WaypointSafetyClassification.Ground_If_Deployed) {
        // CommandScheduler.getInstance().schedule(
        // new SafeDumbTowerToPosition(mElevator, mArm, mRobotState,
        // Waypoints.NEUTRAL));
        // }
        // }
        mElevator.setElevatorState(mElevatorState);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
