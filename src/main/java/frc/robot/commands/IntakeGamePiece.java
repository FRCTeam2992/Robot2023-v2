// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.LEDs;

public class IntakeGamePiece extends CommandBase {
    private Claw mClaw;
    private LEDs mLEDs;
    private RobotState mRobotState;

    private int cyclesAfterBeamBreak;
    private double speed;

    /** Creates a new IntakeGamePiece. */
    public IntakeGamePiece(Claw claw, LEDs leds, RobotState robotState) {
        mClaw = claw;
        mLEDs = leds;
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
                speed = ClawConstants.Intake.Speed.cube;
                break;
            case Cone:
            case Unknown:
            default:
                speed = ClawConstants.Intake.Speed.cone;
        }
        mClaw.setClawSpeed(speed);
        if (mClaw.getBeamBreakTriggered()) {
            cyclesAfterBeamBreak++;
        } else {
            cyclesAfterBeamBreak = 0;
        }
            // Troubleshooting only dashboard
    // SmartDashboard.putNumber("Beam Break Cycles", cyclesAfterBeamBreak);
        }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mClaw.setClawSpeed(0.0);
        if (mClaw.getBeamBreakTriggered()) {
            switch (mRobotState.intakeMode) {
                case Cube:
                    CommandScheduler.getInstance().schedule(
                            new CycleLEDs(mLEDs, Constants.LEDColors.purple, Constants.LEDColors.off).withTimeout(1.5)
                                    .andThen(new SetLEDsColor(mLEDs, Constants.LEDColors.purple)));
                case Cone:
                case Unknown:
                default:
                    CommandScheduler.getInstance().schedule(
                            new CycleLEDs(mLEDs, Constants.LEDColors.yellow, Constants.LEDColors.off).withTimeout(1.5)
                                    .andThen(new SetLEDsColor(mLEDs, Constants.LEDColors.yellow)));
            }
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        switch (mRobotState.intakeMode) {
            case Cube:
                return cyclesAfterBeamBreak >= ClawConstants.Intake.DelayCyclesAfterBeamBreak.cube;
            case Cone:
            case Unknown:
            default:
                return cyclesAfterBeamBreak >= ClawConstants.Intake.DelayCyclesAfterBeamBreak.cone;
        }
    }
}
