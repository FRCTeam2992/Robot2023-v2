// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.subsystems.Claw;

public class IntakeGamePiece extends CommandBase {
  private Claw mClaw;
  private RobotState mRobotState;

  private boolean beamBreakTriggered;
  private int cyclesAfterBeamBreak;

  public static final int DELAY_CYCLES_AFTER_BEAM_BREAK_CONE = 15;
  public static final int DELAY_CYCLES_AFTER_BEAM_BREAK_CUBE = 5;

  /** Creates a new IntakeGamePiece. */
  public IntakeGamePiece(Claw claw, RobotState robotState) {
    mClaw = claw;
    mRobotState = robotState;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mClaw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    beamBreakTriggered = false;
    cyclesAfterBeamBreak = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = mRobotState.intakeMode.clawSpeed;
    mClaw.setClawSpeed(speed);
    if (mClaw.getBeamBreakTriggered()) {
      beamBreakTriggered = true;
    } else if (beamBreakTriggered) {
      cyclesAfterBeamBreak++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    switch (mRobotState.intakeMode) {
      case Cube:
        return cyclesAfterBeamBreak >= IntakeGamePiece.DELAY_CYCLES_AFTER_BEAM_BREAK_CONE;
      case Cone:
      case Unknown:
      default:
        return cyclesAfterBeamBreak >= IntakeGamePiece.DELAY_CYCLES_AFTER_BEAM_BREAK_CONE;
    }
  }
}
