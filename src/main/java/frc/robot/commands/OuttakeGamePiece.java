// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class OuttakeGamePiece extends CommandBase {
    private Claw mClaw;

    private double inSpeed;
    private double outSpeed;

    private int inCyclesDelay;
    private int outCyclesDelay;

    private int inCycleCount;
    private int outCycleCount;

    /** Creates a new OuttakeGamePiece. */
    public OuttakeGamePiece(Claw claw, double inSpeed, int inCyclesDelay, double outSpeed, int outCyclesDelay) {
        mClaw = claw;

        this.inCyclesDelay = inCyclesDelay;
        this.outCyclesDelay = outCyclesDelay;
        this.inSpeed = inSpeed;
        this.outSpeed = outSpeed;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(mClaw);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        inCycleCount = 0;
        outCycleCount = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (inCycleCount < this.inCyclesDelay) {
            mClaw.setClawSpeed(this.inSpeed);
            inCycleCount++;
        } else {
            mClaw.setClawSpeed(-this.outSpeed);
            outCycleCount++;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return outCycleCount >= this.outCyclesDelay;
    }
}
