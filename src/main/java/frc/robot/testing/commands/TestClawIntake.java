// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testing.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.subsystems.Claw;

public class TestClawIntake extends CommandBase {
  private Claw mClaw;
  private RobotState mRobotState;

  private int cyclesAfterBeamBreak;
  private double speed;

  public static final int DELAY_CYCLES_AFTER_BEAM_BREAK_CONE = 12;
  public static final int DELAY_CYCLES_AFTER_BEAM_BREAK_CUBE = 3;

  /** Creates a new TestClawIntakeOuttake. */
  public TestClawIntake(Claw claw, RobotState robotState) {
    mClaw = claw;
    mRobotState = robotState;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mClaw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cyclesAfterBeamBreak = 0;
    speed = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (mRobotState.intakeMode) {
      case Cube:
        speed = SmartDashboard.getNumber("Test Claw Cube In Spd %", 0.0);
        break;
      case Cone:
      case Unknown:
      default:
        speed = SmartDashboard.getNumber("Test Claw Cone In Spd %", 0.0);
    }
    mClaw.setClawSpeed(speed);
    if (mClaw.getBeamBreakTriggered()) {
      cyclesAfterBeamBreak++;
    } else {
      cyclesAfterBeamBreak = 0;
    }
    SmartDashboard.putNumber("Claw Intake Cycles After Beam Break", cyclesAfterBeamBreak);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    switch (mRobotState.intakeMode) {
      case Cube:
        return cyclesAfterBeamBreak >= TestClawIntake.DELAY_CYCLES_AFTER_BEAM_BREAK_CONE;
      case Cone:
      case Unknown:
      default:
        return cyclesAfterBeamBreak >= TestClawIntake.DELAY_CYCLES_AFTER_BEAM_BREAK_CONE;
    }
  }
}
