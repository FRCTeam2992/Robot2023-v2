// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;

public class HoldClaw extends CommandBase {
  /** Creates a new MoveClaw. */
  private Claw mClaw;

  private Timer timer;

  public HoldClaw(Claw subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    mClaw = subsystem;
    timer = new Timer();

    addRequirements(mClaw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    mClaw.setClawSpeed(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() > Constants.ClawConstants.holdPositionMaxTime) {
      mClaw.setClawSpeed(0.0);
    } else {
      if (timer.get() > 0.150) {
        mClaw.holdClaw();
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
