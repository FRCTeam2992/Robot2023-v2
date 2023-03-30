// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.LEDs;

public class ToggleEndgameState extends CommandBase {
    private RobotState mRobotState;
    private LEDs mLEDs;

    /** Creates a new SetEndgameState. */
    public ToggleEndgameState(RobotState robotState, LEDs leds) {
        // Use addRequirements() here to declare subsystem dependencies.
        mRobotState = robotState;
        mLEDs = leds;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        switch (mRobotState.endgameMode) {
            case InEndgame:
                mRobotState.endgameMode = RobotState.EndgameModeState.NotInEndgame;
                switch (mRobotState.intakeMode) {
                    case Cube:
                        CommandScheduler.getInstance().schedule(new SetLEDsColor(mLEDs, Constants.LEDColors.purple));
                        break;
                    case Cone:
                        CommandScheduler.getInstance().schedule(new SetLEDsColor(mLEDs, Constants.LEDColors.yellow));
                        break;
                    case Unknown:
                        CommandScheduler.getInstance().schedule(new SetLEDsColor(mLEDs, Constants.LEDColors.blue));
                        break;
                }
                break;
            case NotInEndgame:
                mRobotState.endgameMode = RobotState.EndgameModeState.InEndgame;
                CommandScheduler.getInstance().schedule(
                    new CycleLEDs(mLEDs, Constants.LEDColors.white, Constants.LEDColors.blue).withTimeout(1.5)
                        .andThen(new SetLEDsColor(mLEDs, Constants.LEDColors.white)));
                break;
        }
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
