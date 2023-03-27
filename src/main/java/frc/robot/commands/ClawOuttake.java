// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.manipulator.Waypoint.OuttakeType;
import frc.robot.RobotState;
import frc.robot.subsystems.Claw;

public class ClawOuttake extends CommandBase {
    private Claw mClaw;
    private RobotState mRobotState;

    private Timer timer;

    public static final int IN_CYCLES_BEFORE_OUT = 5;
    public static final int OUT_CYCLES = 50;

    /** Creates a new TestClawOuttake. */
    public ClawOuttake(Claw claw, RobotState robotState) {
        mClaw = claw;
        mRobotState = robotState;

        timer = new Timer();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(mClaw);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (mRobotState.currentOuttakeType == OuttakeType.Unknown) {
            switch (mRobotState.intakeMode) {
                case Cube:
                    mRobotState.currentOuttakeType = OuttakeType.Assumed_Cube;
                    break;
                case Cone:
                    mRobotState.currentOuttakeType = OuttakeType.Assumed_Cone;
                case Unknown:
                default:
                    mRobotState.currentOuttakeType = OuttakeType.Assumed_Cube;
            }
        }
        mClaw.setClawSpeed(mRobotState.currentOuttakeType.speed);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.get() > mRobotState.currentOuttakeType.time;
    }
}
