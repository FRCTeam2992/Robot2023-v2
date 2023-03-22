// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class MoveClawIntaking extends CommandBase {
  private Claw mClaw;
  private double mSpeed;
  private double mTargetCurrent;

  private LinearFilter lowPass;

  /** Creates a new MoveClawIntaking. */
  public MoveClawIntaking(Claw subsystem, double speed, double targetCurrent) {
    // Use addRequirements() here to declare subsystem dependencies.
    mClaw = subsystem;
    mSpeed = speed;
    mTargetCurrent = targetCurrent;

    lowPass = LinearFilter.movingAverage(3);

    addRequirements(mClaw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mClaw.setClawSpeed(mSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mClaw.getClawCurrent() < mTargetCurrent;
  }
}
