// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.LEDs;

public class LEDsToDefaultColor extends CommandBase {
  private LEDs mLEDs;
  private RobotState mRobotState;
  
  /** Creates a new LEDsToDefaultColor. */
  public LEDsToDefaultColor(LEDs leds, RobotState robotState) {
    mLEDs = leds;
    mRobotState = robotState;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mLEDs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (mRobotState.isInEndgameMode()) {
      mLEDs.setLEDStripColor(Constants.LEDColors.white);
    } else {
      switch (mRobotState.intakeMode) {
        case Cube:
          mLEDs.setLEDStripColor(Constants.LEDColors.purple);
          break;
        case Cone:
          mLEDs.setLEDStripColor(Constants.LEDColors.yellow);
          break;
        case Unknown:
          mLEDs.setLEDStripColor(Constants.LEDColors.blue);
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
