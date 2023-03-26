// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testing.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotState;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.subsystems.Elevator;

public class TestElevatorPID extends CommandBase {
    private Elevator mElevator;
    private RobotState mRobotState;

    /** Creates a new TestTowerSafeMove. */
    public TestElevatorPID(Elevator elevator, RobotState robotState) {
        // Use addRequirements() here to declare subsystem dependencies.
        mElevator = elevator;
        mRobotState = robotState;
        addRequirements(elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        double height;

        height = SmartDashboard.getNumber("ElevTestMoveHeight", 20);
        System.out.println("ElevatorTestMove running to " + height);

        CommandScheduler.getInstance().schedule(new SetElevatorPosition(mElevator, height));
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
