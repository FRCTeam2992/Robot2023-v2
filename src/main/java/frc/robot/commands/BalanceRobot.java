// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class BalanceRobot extends CommandBase {
    private Drivetrain mDrivetrain;
    private LinearFilter lowPass;
    private BalanceStateMachine stateMachine;   
    private double priorPitch;
    private double currentPitch;
    private double currentPitchDelta;
    private int correctionsCompleted;
    private double correctionSpeed;

    /** Creates a new BalanceRobot. */
    public BalanceRobot(Drivetrain driveTrain) {
        // Use addRequirements() here to declare subsystem dependencies.
        mDrivetrain = driveTrain;
        lowPass = LinearFilter.movingAverage(3);
        stateMachine = new BalanceStateMachine();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        priorPitch = currentPitch = mDrivetrain.getRobotPitch();
        lowPass.reset();
        correctionsCompleted = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        priorPitch = currentPitch;
        currentPitch = lowPass.calculate(mDrivetrain.getRobotPitch());
        currentPitchDelta = currentPitch - priorPitch;

        stateMachine.execute(currentPitch, currentPitchDelta);

        switch (stateMachine.state) {
            case ForwardCorrection:
                this.correctionSpeed = getCorrectionSpeed(true);
                mDrivetrain.moveRobotFrontBack(true, this.correctionSpeed);
                break;
            case ReverseCorrection:
                this.correctionSpeed = getCorrectionSpeed(false);
                mDrivetrain.moveRobotFrontBack(false, this.correctionSpeed);
                break;
            case Complete:
                correctionsCompleted++;
                break;
            default:
                break;
        }
    }

    private double getCorrectionSpeed(boolean forward) {
        if (forward) {
            return DrivetrainConstants.balanceMoveSpeed * getCorrectionFactor();
        } else {
            return (DrivetrainConstants.balanceMoveSpeed - .1) * getCorrectionFactor();
        }
    }

    private double getCorrectionFactor() {
        double correctionFactor = 1.0;
        if (this.correctionsCompleted > 0) {
            correctionFactor *= 0.5 * Math.max(0.3, 1 - 0.1 * this.correctionsCompleted);
        }
        return correctionFactor;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.stateMachine.state == BalanceStateMachine.BalanceModeState.Balanced;
    }

    public static class BalanceStateMachine {
        public final int WAIT_CYCLES_INTOLERANCE = 250;
        public final int WAIT_CYCLES_NEXT_CORRECTION = 150;
        
        public enum BalanceModeState {
            Uncorrected,
            ForwardCorrection,
            ReverseCorrection,
            Waiting,
            Balanced,
            Complete
        }

        public BalanceModeState state;
        private int waitCycleCounter;
        private int balancedCycleCounter;
        public int correctionCount;

        public BalanceStateMachine() {
            this.state = BalanceModeState.Uncorrected;
            this.correctionCount = 0;
            this.waitCycleCounter = 0;
            this.balancedCycleCounter = 0;
        }

        public void execute(double currentPitch, double currentPitchDelta) {
            switch (this.state) {
                case Balanced:
                    checkStillBalanced(currentPitch, currentPitchDelta);
                    break;
                case Uncorrected:
                    maybeCorrect(currentPitch, currentPitchDelta);
                    break;
                case ForwardCorrection:
                    this.balancedCycleCounter = 0;
                    maybeCompleteCorrection(currentPitch, currentPitchDelta);
                    break;
                case ReverseCorrection:
                    this.balancedCycleCounter = 0;
                    maybeCompleteCorrection(currentPitch, currentPitchDelta);
                    break;
                case Waiting:
                    waitCycle();
                    break;
                case Complete:
                    waitForNextCorrection();
                    break;                                        
            }
        }

        public boolean canMakeCorrection() {
            return this.state == BalanceModeState.Uncorrected ||
                this.state == BalanceModeState.Waiting;
        }

        public boolean needsForwardCorrection(double currentPitch, double currentPitchDelta) {
            double pitchTolerance = DrivetrainConstants.pitchTolerance;
            return (currentPitch > pitchTolerance) && !chargeStationMoving(currentPitchDelta);
        }

        public boolean needsReverseCorrection(double currentPitch, double currentPitchDelta) {
            double pitchTolerance = DrivetrainConstants.pitchTolerance;
            return (currentPitch < -pitchTolerance) && !chargeStationMoving(currentPitchDelta);
        }

        public boolean chargeStationMoving(double currentPitchDelta) {
            double pitchDeltaTolerance = DrivetrainConstants.pitchDeltaTolerance;
            return (currentPitchDelta > -pitchDeltaTolerance) && (currentPitchDelta < pitchDeltaTolerance);
        }

        public boolean chargeStationLevel(double currentPitch) {
            double pitchTolerance = DrivetrainConstants.pitchTolerance;
            return (currentPitch > -pitchTolerance) && (currentPitch < pitchTolerance);
        }

        public void makeForwardCorrection() {
            if (canMakeCorrection()) {
                this.state = BalanceModeState.ForwardCorrection;
            }
        }

        public void makeReverseCorrection() {
            if (canMakeCorrection()) {
                this.state = BalanceModeState.ReverseCorrection;
            }
        }

        public void maybeCorrect(double currentPitch, double currentPitchDelta) {
            if (needsForwardCorrection(currentPitch, currentPitchDelta)) {
                makeForwardCorrection();
            } else if (needsReverseCorrection(currentPitch, currentPitchDelta)) {
                makeReverseCorrection();
            }
        }

        public void maybeCompleteCorrection(double currentPitch, double currentPitchDelta) {
            if (chargeStationMoving(currentPitchDelta) && chargeStationLevel(currentPitch)) {
                this.state = BalanceModeState.Complete;
            } else {
                this.state = BalanceModeState.Uncorrected;
            }
        }

        public void waitForNextCorrection() {
            this.balancedCycleCounter = 0;
            this.waitCycleCounter = 0;
            this.state = BalanceModeState.Waiting;
        }

        public void waitCycle() {
            this.waitCycleCounter++;
            if (this.waitCycleCounter > this.WAIT_CYCLES_NEXT_CORRECTION) {
                this.state = BalanceModeState.Balanced;
            }
        }

        public void finish() {
            this.state = BalanceModeState.Balanced;
        }

        public void checkStillBalanced(double currentPitch, double currentPitchDelta) {
            if (chargeStationMoving(currentPitchDelta) || !chargeStationLevel(currentPitch)) {
                this.state = BalanceModeState.Uncorrected;
            } else {
                this.balancedCycleCounter++;
                if (this.balancedCycleCounter > this.WAIT_CYCLES_INTOLERANCE) {
                    finish();
                }
            }
        }
    }
}
