// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.leds.Color;
import frc.robot.subsystems.LEDs;

public class CycleLEDs extends CommandBase {
    private LEDs mLEDs;
    private Color color1;
    private Color color2;

    private int ledsLoopCounter;
    private int ledsFrameCounter;

    /** Creates a new CycleLEDs. */
    public CycleLEDs(LEDs leds, Color color1, Color color2) {
        mLEDs = leds;
        this.color1 = color1;
        this.color2 = color2;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(mLEDs);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ledsLoopCounter = 0;
        ledsFrameCounter = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (ledsLoopCounter == 5) {
            mLEDs.showNextCycleColor(color1, color2, ledsFrameCounter);
            if (ledsFrameCounter == 6) {
                ledsFrameCounter = 0;
            }
            ledsLoopCounter = 0;
            ledsFrameCounter++;
        }
        ledsLoopCounter++;
    }

    // Called when the command is scheduled to run during disabled mode
    // Return of true allows the command to run
    @Override
    public boolean runsWhenDisabled() {
        return true;
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
