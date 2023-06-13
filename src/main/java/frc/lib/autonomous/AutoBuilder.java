// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.autonomous;

import java.util.HashMap;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.manipulator.Waypoint;
import frc.lib.manipulator.Waypoint.OuttakeType;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.GridTargetingPosition;
import frc.robot.RobotState.IntakeModeState;
import frc.robot.commands.BalanceRobotPID;
import frc.robot.commands.ClawOuttake;
import frc.robot.commands.DeployElevator;
import frc.robot.commands.HoldArm;
import frc.robot.commands.HoldClaw;
import frc.robot.commands.IntakeGamePiece;
import frc.robot.commands.MoveArmToPoint;
import frc.robot.commands.MoveClaw;
import frc.robot.commands.MoveClawIntaking;
import frc.robot.commands.SetLimeLightOdometryUpdates;
import frc.robot.commands.StopClaw;
import frc.robot.commands.groups.FollowTrajectoryCommand;
import frc.robot.commands.groups.SafeDumbTowerToPosition;
import frc.robot.commands.groups.UnsafeMoveTowerToPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;

/** Add your docs here. */
public class AutoBuilder {
    private RobotState mRobotState;
    private Drivetrain mDrivetrain;
    private Elevator mElevator;
    private Arm mArm;
    private Claw mClaw;
    private LEDs mLEDs;

    private SendableChooser<AutoStartPosition> autoStartChooser;
    private SendableChooser<AutoSequence> autoSequenceChooser;
    private SendableChooser<AutoPreloadScore> autoPreloadScoreChooser;

    private HashMap<String, Command> eventMap = new HashMap<>();

    public AutoBuilder(RobotState robotState, Drivetrain drivetrain, Elevator elevator,
            Arm arm, Claw claw, LEDs leds) {
        mRobotState = robotState;
        mDrivetrain = drivetrain;
        mElevator = elevator;
        mArm = arm;
        mClaw = claw;
        mLEDs = leds;

        eventMap.put("SetIntakeModeCube", new InstantCommand(() -> mRobotState.intakeMode = IntakeModeState.Cube));
        eventMap.put("DeployElevator", new DeployElevator(mElevator, mArm, mRobotState, ElevatorState.Deployed));
        eventMap.put("UndeployElevator", new DeployElevator(mElevator, mArm, mRobotState, ElevatorState.Undeployed));
        eventMap.put("TowerMoveHighRight", new ScheduleCommand(new SafeDumbTowerToPosition(
                mElevator, mArm, mRobotState,
                GridTargetingPosition.HighRight.towerWaypoint)).asProxy());
        eventMap.put("TowerMoveHighCenter", new ScheduleCommand(new SafeDumbTowerToPosition(
                mElevator, mArm, mRobotState,
                GridTargetingPosition.HighCenter.towerWaypoint)).asProxy());
        eventMap.put("TowerMoveMidCenter", new ScheduleCommand(new SafeDumbTowerToPosition(
                mElevator, mArm, mRobotState,
                GridTargetingPosition.MidCenter.towerWaypoint)).asProxy());
        eventMap.put("TowerMoveThrowCube", new ScheduleCommand(new SafeDumbTowerToPosition(
                mElevator, mArm, mRobotState,
                GridTargetingPosition.ThrowCube.towerWaypoint)).asProxy());
        eventMap.put("TowerMoveGroundIntake", new ScheduleCommand(new SafeDumbTowerToPosition(
                mElevator, mArm, mRobotState,
                Constants.TowerConstants.cubeGroundIntake)).asProxy());
        eventMap.put("TowerMoveHiGroundIntake", new ScheduleCommand(new SafeDumbTowerToPosition(
                mElevator, mArm, mRobotState,
                Constants.TowerConstants.cubeWall3GroundIntake)).asProxy());

        eventMap.put("TowerMoveStowed", new ScheduleCommand(new SafeDumbTowerToPosition(
                mElevator, mArm, mRobotState,
                Constants.TowerConstants.normal)).asProxy());
        eventMap.put("TowerMoveLoadStation", new ScheduleCommand(new SafeDumbTowerToPosition(
                mElevator, mArm, mRobotState,
                Constants.TowerConstants.singleLoadStation)).asProxy());
        eventMap.put("TowerMoveUnsafeRearSafePoint", new ScheduleCommand(new SafeDumbTowerToPosition(
                mElevator, mArm, mRobotState,
                Constants.TowerConstants.rearSafePoint)).asProxy());

        eventMap.put("StartCubeIntake", new IntakeGamePiece(mClaw, mLEDs, mRobotState));
        eventMap.put("StartCubeOuttake", new ClawOuttake(mClaw, mRobotState));
        eventMap.put("StopClaw", new StopClaw(mClaw));
        eventMap.put("EndIntake", new MoveClaw(mClaw, 0.2));
        eventMap.put("StopLimelight", new SetLimeLightOdometryUpdates(mRobotState, mDrivetrain, false));
        eventMap.put("StartLimelight", new SetLimeLightOdometryUpdates(mRobotState, mDrivetrain, true));
        eventMap.put("StartCubeLaunch", new MoveArmToPoint(mArm, mClaw, mDrivetrain, 1.0, 10.0, 30.0, -5.0));
    }

    public void setupAutoSelector() {
        // Setup choosers for start position
        autoStartChooser = new SendableChooser<>();
        autoStartChooser.addOption(AutoStartPosition.LoadStationEnd.description,
                AutoStartPosition.LoadStationEnd);
        autoStartChooser.addOption(AutoStartPosition.LoadStationCube.description, AutoStartPosition.LoadStationCube);
        autoStartChooser.addOption(AutoStartPosition.WallCube.description, AutoStartPosition.WallCube);
        autoStartChooser.addOption(AutoStartPosition.CenterLoadStationSide.description,
                AutoStartPosition.CenterLoadStationSide);
        autoStartChooser.addOption(AutoStartPosition.CenterWallSide.description, AutoStartPosition.CenterWallSide);
        autoStartChooser.setDefaultOption(AutoStartPosition.WallEnd.description,
                AutoStartPosition.WallEnd);

        SmartDashboard.putData("Auto Start Position", autoStartChooser);

        // Setup chooser for preload scoring
        autoPreloadScoreChooser = new SendableChooser<>();
        autoPreloadScoreChooser.addOption(AutoPreloadScore.No_Preload.description, AutoPreloadScore.No_Preload);
        // autoPreloadScoreChooser.addOption(AutoPreloadScore.Mid_Cube.description,
        // AutoPreloadScore.Mid_Cube);
        autoPreloadScoreChooser.addOption(AutoPreloadScore.Mid_Cube_Reversed.description,
                AutoPreloadScore.Mid_Cube_Reversed);
        autoPreloadScoreChooser.setDefaultOption(AutoPreloadScore.Hi_Cone.description, AutoPreloadScore.Hi_Cone);

        SmartDashboard.putData("Preload Score?", autoPreloadScoreChooser);

        // Setup chooser for auto sequence
        autoSequenceChooser = new SendableChooser<>();
        autoSequenceChooser.setDefaultOption(AutoSequence.Do_Nothing.description, AutoSequence.Do_Nothing);
        autoSequenceChooser.addOption(AutoSequence.SideMobilityOnly.description,
                AutoSequence.SideMobilityOnly);
        autoSequenceChooser.addOption(AutoSequence.SideMobilityBalance.description,
                AutoSequence.SideMobilityBalance);
        autoSequenceChooser.addOption(AutoSequence.SideIntakeBalance.description,
                AutoSequence.SideIntakeBalance);
        autoSequenceChooser.addOption(AutoSequence.Side2Scores.description, AutoSequence.Side2Scores);
        autoSequenceChooser.addOption(AutoSequence.Side3Scores.description, AutoSequence.Side3Scores);
        autoSequenceChooser.addOption(AutoSequence.CenterBalance.description, AutoSequence.CenterBalance);
        autoSequenceChooser.addOption(AutoSequence.CenterIntakeBalance.description, AutoSequence.CenterIntakeBalance);
        autoSequenceChooser.addOption(AutoSequence.Side2ScoreBalance.description, AutoSequence.Side2ScoreBalance);

        SmartDashboard.putData("Auto Sequence", autoSequenceChooser);

    }

    public AutoStartPosition getAutoStartPosition() {
        return autoStartChooser.getSelected();
    }

    public AutoPreloadScore getAutoPreloadScore() {
        return autoPreloadScoreChooser.getSelected();
    }

    public AutoSequence getAutoSequence() {
        return autoSequenceChooser.getSelected();
    }

    public boolean autoStartCompatible() {
        // Returns true if the Auto Start Position is valid for the current selected
        // sequence
        return autoSequenceChooser.getSelected().allowedStartPositions.contains(
                autoStartChooser.getSelected());
    }

    private Command setupAutoInitialScoreCommand() {
        Command initialScoreCommand;
        Pose2d startingPose = getAutoStartPosition().getStartPose();
        if (startingPose == null) {
            return new InstantCommand();
        }
        initialScoreCommand = new InstantCommand(() -> mDrivetrain.resetOdometryToPose(startingPose));
        switch (getAutoPreloadScore()) {
            case Hi_Cone:
                initialScoreCommand = initialScoreCommand
                        .andThen(new DeployElevator(mElevator, mArm, mRobotState, ElevatorState.Deployed)
                                .andThen(new InstantCommand(() -> mRobotState.currentOuttakeType = OuttakeType.Hi_Cone))
                                .andThen(new WaitCommand(0.2))
                                .andThen(new SafeDumbTowerToPosition(
                                        mElevator, mArm, mRobotState, GridTargetingPosition.HighRight.towerWaypoint)
                                        .withTimeout(1.2)
                                        .alongWith(new WaitCommand(1.0)))
                                .andThen(new WaitCommand(0.7))
                                .andThen(new ClawOuttake(mClaw, mRobotState).withTimeout(0.6)));
                break;
            case Mid_Cube:
                initialScoreCommand = initialScoreCommand
                        .andThen(new InstantCommand(() -> {
                            mRobotState.currentOuttakeType = OuttakeType.Mid_Cube;
                            mRobotState.intakeMode = IntakeModeState.Cube;
                        }))
                        .andThen(new SafeDumbTowerToPosition(mElevator, mArm, mRobotState,
                                GridTargetingPosition.MidCenter.towerWaypoint)
                                .withTimeout(0.3)
                                .raceWith(new MoveClaw(mClaw, 0.5)))
                        .andThen(new ClawOuttake(mClaw, mRobotState).withTimeout(0.5));
                break;
            case Mid_Cube_Reversed:
                initialScoreCommand = initialScoreCommand
                        .andThen(new InstantCommand(() -> {
                            mRobotState.currentOuttakeType = OuttakeType.Unknown;
                            mRobotState.intakeMode = IntakeModeState.Cube;
                        }))
                        .andThen(new UnsafeMoveTowerToPosition(mElevator, mArm, Constants.TowerConstants.rearSafePoint)
                                .asProxy()
                                .withTimeout(0.5)
                                .raceWith(new MoveClaw(mClaw, 0.5)))
                        .andThen(new UnsafeMoveTowerToPosition(mElevator, mArm,
                                Constants.TowerConstants.rearMidThrowCube).asProxy()
                                .withTimeout(0.5)
                                .raceWith(new MoveClaw(mClaw, 0.5)))
                        .andThen(new WaitCommand(0.1))
                        .andThen(new ClawOuttake(mClaw, mRobotState).withTimeout(0.4));
                break;
            case No_Preload:
            default:
        }
        return initialScoreCommand;
    }

    private Command setupAutoPathFollowCommand(boolean isFirstPath) {
        Command followCommand = new DeployElevator(mElevator, mArm, mRobotState, ElevatorState.Undeployed)
                .alongWith(new WaitCommand(0.1).andThen(new SafeDumbTowerToPosition(mElevator, mArm, mRobotState,
                        Constants.TowerConstants.normal)).withTimeout(0.5));
        isFirstPath = true;
        switch (getAutoSequence()) {
            case Do_Nothing:
                break;
            case SideMobilityOnly:
                if (getAutoStartPosition() == AutoStartPosition.LoadStationEnd) {
                    for (PathPlannerTrajectory path : AutonomousTrajectory.LoadStationMobility.trajectoryGroup) {
                        followCommand = followCommand.andThen(new FollowPathWithEvents(
                                new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath),
                                path.getMarkers(),
                                eventMap));
                        isFirstPath = false; // Make sure it's false for subsequent paths
                    }
                } else if (getAutoStartPosition() == AutoStartPosition.WallEnd) {
                    for (PathPlannerTrajectory path : AutonomousTrajectory.WallMobility.trajectoryGroup) {
                        followCommand = followCommand.andThen(new FollowPathWithEvents(
                                new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath),
                                path.getMarkers(),
                                eventMap));
                        isFirstPath = false; // Make sure it's false for subsequent paths
                    }
                }
                break;
            case SideIntakeBalance:
                if (getAutoStartPosition() == AutoStartPosition.LoadStationEnd) {
                    for (PathPlannerTrajectory path : AutonomousTrajectory.LoadStationIntakeBalance.trajectoryGroup) {
                        followCommand = followCommand.andThen(new FollowPathWithEvents(
                                new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath),
                                path.getMarkers(),
                                eventMap));
                        isFirstPath = false; // Make sure it's false for subsequent paths
                    }
                } else if (getAutoStartPosition() == AutoStartPosition.WallEnd) {
                    for (PathPlannerTrajectory path : AutonomousTrajectory.WallIntakeBalance.trajectoryGroup) {
                        followCommand = followCommand.andThen(new FollowPathWithEvents(
                                new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath),
                                path.getMarkers(),
                                eventMap));
                        isFirstPath = false; // Make sure it's false for subsequent paths
                    }
                }
                followCommand = followCommand.andThen(new BalanceRobotPID(mDrivetrain));
                break;
            case Side2Scores:
                if (getAutoStartPosition() == AutoStartPosition.LoadStationEnd) {
                    for (PathPlannerTrajectory path : AutonomousTrajectory.LoadStation2Scores.trajectoryGroup) {
                        followCommand = followCommand.andThen(new FollowPathWithEvents(
                                new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath),
                                path.getMarkers(),
                                eventMap));
                        isFirstPath = false; // Make sure it's false for subsequent paths
                    }
                } else if (getAutoStartPosition() == AutoStartPosition.WallEnd) {
                    for (PathPlannerTrajectory path : AutonomousTrajectory.Wall2Scores.trajectoryGroup) {
                        followCommand = followCommand.andThen(new FollowPathWithEvents(
                                new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath),
                                path.getMarkers(),
                                eventMap));
                        isFirstPath = false; // Make sure it's false for subsequent paths
                    }
                }
                followCommand = followCommand
                        .andThen(new InstantCommand(() -> {
                            mDrivetrain.stopDrive();
                            mRobotState.currentOuttakeType = OuttakeType.Hi_Cube;
                        }))
                        .andThen(new WaitCommand(1.0))
                        .andThen(new ClawOuttake(mClaw, mRobotState).withTimeout(1.0))
                        .andThen(new DeployElevator(mElevator, mArm, mRobotState, ElevatorState.Undeployed))
                        .andThen(new SafeDumbTowerToPosition(mElevator, mArm, mRobotState,
                                Constants.TowerConstants.normal));
                break;
            case Side3Scores:
                followCommand = new InstantCommand();
                if (getAutoStartPosition() == AutoStartPosition.LoadStationCube) {
                    for (PathPlannerTrajectory path : AutonomousTrajectory.LoadStation3ScoresPart1.trajectoryGroup) {
                        followCommand = followCommand.andThen(new FollowPathWithEvents(
                                new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath),
                                path.getMarkers(),
                                eventMap));
                        isFirstPath = false; // Make sure it's false for subsequent paths
                    }
                    followCommand = followCommand.andThen(new InstantCommand(() -> mDrivetrain.stopDrive()))
                            .andThen(new MoveClaw(mClaw, Waypoint.OuttakeType.Rear_Low_Cube.speed).withTimeout(0.8));
                    for (PathPlannerTrajectory path : AutonomousTrajectory.LoadStation3ScoresPart2.trajectoryGroup) {
                        followCommand = followCommand.andThen(new FollowPathWithEvents(
                                new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath),
                                path.getMarkers(),
                                eventMap));
                    }
                } else if (getAutoStartPosition() == AutoStartPosition.WallCube) {
                    for (PathPlannerTrajectory path : AutonomousTrajectory.Wall3ScoresPart1.trajectoryGroup) {
                        followCommand = followCommand.andThen(new FollowPathWithEvents(
                                new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath),
                                path.getMarkers(),
                                eventMap));
                        isFirstPath = false; // Make sure it's false for subsequent paths
                    }
                    followCommand = followCommand.andThen(new InstantCommand(() -> {
                        mDrivetrain.stopDrive();
                        mRobotState.currentOuttakeType = OuttakeType.Rear_Low_Cube;
                    }))
                            .andThen(new MoveClaw(mClaw, Waypoint.OuttakeType.Rear_Low_Cube.speed).withTimeout(0.4));
                    for (PathPlannerTrajectory path : AutonomousTrajectory.Wall3ScoresPart2.trajectoryGroup) {
                        followCommand = followCommand.andThen(new FollowPathWithEvents(
                                new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath),
                                path.getMarkers(),
                                eventMap));
                    }
                }

                followCommand = followCommand.andThen(new InstantCommand(() -> mDrivetrain.stopDrive()))
                        .andThen(new MoveClaw(mClaw, Waypoint.OuttakeType.Rear_Low_Cube.speed));
                break;
            case Side2ScoreBalance:
                followCommand = new DeployElevator(mElevator, mArm, mRobotState, ElevatorState.Undeployed);
                if (getAutoStartPosition() == AutoStartPosition.LoadStationEnd) {
                    for (PathPlannerTrajectory path : AutonomousTrajectory.LoadStation2ScoreBalance.trajectoryGroup) {
                        followCommand = followCommand.andThen(new FollowPathWithEvents(
                                new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath),
                                path.getMarkers(),
                                eventMap));
                        isFirstPath = false; // Make sure it's false for subsequent paths
                    }
                } else if (getAutoStartPosition() == AutoStartPosition.WallEnd) {
                    for (PathPlannerTrajectory path : AutonomousTrajectory.Wall2ScoreBalance.trajectoryGroup) {
                        followCommand = followCommand.andThen(new FollowPathWithEvents(
                                new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath),
                                path.getMarkers(),
                                eventMap));
                        isFirstPath = false; // Make sure it's false for subsequent paths
                    }
                }
                followCommand = followCommand.andThen(new BalanceRobotPID(mDrivetrain));
                break;
            case SideMobilityBalance:
                if (getAutoStartPosition() == AutoStartPosition.LoadStationEnd) {
                    for (PathPlannerTrajectory path : AutonomousTrajectory.LoadStationMobilityBalance.trajectoryGroup) {
                        followCommand = followCommand.andThen(new FollowPathWithEvents(
                                new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath),
                                path.getMarkers(),
                                eventMap));
                        isFirstPath = false; // Make sure it's false for subsequent paths
                    }
                } else if (getAutoStartPosition() == AutoStartPosition.WallEnd) {
                    for (PathPlannerTrajectory path : AutonomousTrajectory.WallMobilityBalance.trajectoryGroup) {
                        followCommand = followCommand.andThen(new FollowPathWithEvents(
                                new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath),
                                path.getMarkers(),
                                eventMap));
                        isFirstPath = false; // Make sure it's false for subsequent paths
                    }
                }
                followCommand = followCommand.andThen(new BalanceRobotPID(mDrivetrain));
                break;
            case CenterBalance:
                followCommand = followCommand.andThen(new SetLimeLightOdometryUpdates(mRobotState, mDrivetrain, false));
                if (getAutoStartPosition() == AutoStartPosition.CenterLoadStationSide) {
                    for (PathPlannerTrajectory path : AutonomousTrajectory.CenterBalanceLoadStationSide.trajectoryGroup) {
                        followCommand = followCommand.andThen(new FollowPathWithEvents(
                                new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath),
                                path.getMarkers(),
                                eventMap));
                        isFirstPath = false; // Make sure it's false for subsequent paths
                    }
                } else if (getAutoStartPosition() == AutoStartPosition.CenterWallSide) {
                    for (PathPlannerTrajectory path : AutonomousTrajectory.CenterBalanceWallSide.trajectoryGroup) {
                        followCommand = followCommand.andThen(new FollowPathWithEvents(
                                new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath),
                                path.getMarkers(),
                                eventMap));
                        isFirstPath = false; // Make sure it's false for subsequent paths
                    }
                }
                followCommand = followCommand.andThen(new BalanceRobotPID(mDrivetrain));
                break;
            case CenterIntakeBalance:
                followCommand = followCommand.andThen(new SetLimeLightOdometryUpdates(mRobotState, mDrivetrain, false));
                if (getAutoStartPosition() == AutoStartPosition.CenterLoadStationSide) {
                    for (PathPlannerTrajectory path : AutonomousTrajectory.CenterIntakeBalanceLoadStationSide.trajectoryGroup) {
                        followCommand = followCommand.andThen(new FollowPathWithEvents(
                                new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath),
                                path.getMarkers(),
                                eventMap));
                        isFirstPath = false; // Make sure it's false for subsequent paths
                    }
                } else if (getAutoStartPosition() == AutoStartPosition.CenterWallSide) {
                    for (PathPlannerTrajectory path : AutonomousTrajectory.CenterIntakeBalanceWallSide.trajectoryGroup) {
                        followCommand = followCommand.andThen(new FollowPathWithEvents(
                                new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath),
                                path.getMarkers(),
                                eventMap));
                        isFirstPath = false; // Make sure it's false for subsequent paths
                    }
                }
                followCommand = followCommand.andThen(new BalanceRobotPID(mDrivetrain));
                break;
            case Center2ScoreBalance:
                followCommand = followCommand.andThen(new SetLimeLightOdometryUpdates(mRobotState, mDrivetrain, false));
                if (getAutoStartPosition() == AutoStartPosition.CenterLoadStationSide) {
                    for (PathPlannerTrajectory path : AutonomousTrajectory.Center2ScoreBalanceLoadStationSide.trajectoryGroup) {
                        followCommand = followCommand.andThen(new FollowPathWithEvents(
                                new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath),
                                path.getMarkers(),
                                eventMap));
                        isFirstPath = false; // Make sure it's false for subsequent paths
                    }
                } else if (getAutoStartPosition() == AutoStartPosition.CenterWallSide) {
                    for (PathPlannerTrajectory path : AutonomousTrajectory.Center2ScoreBalanceWallSide.trajectoryGroup) {
                        followCommand = followCommand.andThen(new FollowPathWithEvents(
                                new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath),
                                path.getMarkers(),
                                eventMap));
                        isFirstPath = false; // Make sure it's false for subsequent paths
                    }
                }
                followCommand = followCommand.andThen(new BalanceRobotPID(mDrivetrain));
                break;
        }
        return followCommand;
    }

    public Command buildAutoCommand() {
        Command autoPathCommand = null;
        Command initialScoreCommand = null;
        Command afterInitialScoreCommand = null;

        // Setup the initial preload scoring path and command sequence
        initialScoreCommand = setupAutoInitialScoreCommand();

        if (!autoStartCompatible()) {
            // We have incompatible starting position for sequence.
            // Run only the initial score command, which in the case of
            // No_Preload, just resets odometry and stops.
            return initialScoreCommand;
        } else {
            // Starting position is compatible, so setup the path following command,
            // then build a parallel group to move from scoring position while driving
            if (getAutoPreloadScore() != AutoPreloadScore.No_Preload) {
                autoPathCommand = setupAutoPathFollowCommand(false);
                afterInitialScoreCommand = autoPathCommand;
            } else {
                // In the case of No_Preload, we didn't score, so no arm/elevator/claw
                // reset is needed, and we can just follow the path directly.
                // The path will be our first path, since no initial path is needed if
                // we don't score a preload.
                autoPathCommand = setupAutoPathFollowCommand(true);
                afterInitialScoreCommand = autoPathCommand;
            }

            // If we've completed the above, we should always have a Command object for
            // both initialScoreCommand and afterInitialScoreCommand (either or both of
            // which may be just a dummy InstantCommand that does nothing), so we can now
            // return a sequence of those Commands.
            return initialScoreCommand.andThen(afterInitialScoreCommand);
        }
    }

}
