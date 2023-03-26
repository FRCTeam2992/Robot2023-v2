// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.manipulator.Waypoint.OuttakeType;
import frc.robot.RobotState;
import frc.robot.subsystems.Arm;

public class MoveArm extends CommandBase {
  /** Creates a new MoveArm. */
  private Arm mArm;
  private RobotState mRobotState;

  private double mArmSpeed;

  public MoveArm(Arm subsystem, RobotState robotState, double armspeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    mArm = subsystem;
    mRobotState = robotState;
    mArmSpeed = armspeed;

    addRequirements(mArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mArm.setArmSpeed(mArmSpeed);
    // mRobotState.currentOuttakeType = OuttakeType.Unknown;
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mArm.setArmSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
