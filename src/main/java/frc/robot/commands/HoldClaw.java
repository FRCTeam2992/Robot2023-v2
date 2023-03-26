// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        // mClaw.setClawSpeed(0.0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putNumber("Claw Hold Timer", timer.get());
        if (timer.get() > Constants.ClawConstants.holdPositionMaxTime) {
            SmartDashboard.putBoolean("Claw Holding PID", false);
            mClaw.setClawSpeed(0.0);
        } else {
            SmartDashboard.putBoolean("Claw Holding PID", true);
            mClaw.holdClaw();
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
