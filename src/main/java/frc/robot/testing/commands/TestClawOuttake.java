// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testing.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.subsystems.Claw;

public class TestClawOuttake extends CommandBase {
    private Claw mClaw;
    private RobotState mRobotState;

    private int inCycles;
    private int outCycles;
    private double inSpeed;
    private double outSpeed;

    public static final int IN_CYCLES_BEFORE_OUT = 5;
    public static final int OUT_CYCLES = 50;

    /** Creates a new TestClawOuttake. */
    public TestClawOuttake(Claw claw, RobotState robotState) {
        mClaw = claw;
        mRobotState = robotState;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(mClaw);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        inCycles = 0;
        outCycles = 0;

        inSpeed = 0.0;
        outSpeed = 0.0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        switch (mRobotState.intakeMode) {
            case Cube:
                inSpeed = SmartDashboard.getNumber("Test Claw Cube In Spd %", 0.0);
                outSpeed = SmartDashboard.getNumber("Test Claw Cube Out Spd %", 0.0);
                break;
            case Cone:
            case Unknown:
            default:
                inSpeed = SmartDashboard.getNumber("Test Claw Cone In Spd %", 0.0);
                outSpeed = SmartDashboard.getNumber("Test Claw Cone Out Spd %", 0.0);
        }

        if (inCycles < TestClawOuttake.IN_CYCLES_BEFORE_OUT) {
            mClaw.setClawSpeed(inSpeed);
            inCycles++;
        } else {
            mClaw.setClawSpeed(-outSpeed);
            outCycles++;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return outCycles >= TestClawOuttake.OUT_CYCLES;
    }
}
