// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.commands.IntakeGamePiece;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoGroundIntakeCube extends ParallelCommandGroup {
    public AutoGroundIntakeCube(Elevator elevator, Arm arm, Claw claw, RobotState robotState) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new SafeDumbTowerToPosition(
                        elevator, arm, robotState, Constants.TowerConstants.cubeGroundIntake).asProxy(),
                        new InstantCommand(() -> {
                            robotState.intakeMode = RobotState.IntakeModeState.Cube;
                        }).andThen(new IntakeGamePiece(claw, robotState)));
    }
}