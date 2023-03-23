// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
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
        addRequirements(mDrivetrain);
        lowPass = LinearFilter.movingAverage(1);
        stateMachine = new BalanceStateMachine();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        priorPitch = currentPitch = mDrivetrain.getRobotPitch();
        lowPass.reset();
        correctionsCompleted = 0;
        mDrivetrain.setDriveNeutralMode(NeutralMode.Brake);
        this.stateMachine.state = BalanceStateMachine.BalanceModeState.Uncorrected;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        priorPitch = currentPitch;
        currentPitch = lowPass.calculate(mDrivetrain.getRobotPitch());
        currentPitchDelta = currentPitch - priorPitch;

        stateMachine.execute(currentPitch, currentPitchDelta);
        SmartDashboard.putString("Balance State", this.stateMachine.state.toString());
        SmartDashboard.putNumber("Robot pitch lowpassed", currentPitch);
        SmartDashboard.putNumber("Robot pitchDelta", currentPitchDelta);
        SmartDashboard.putNumber("Balance Corrections Completed", this.correctionsCompleted);

        switch (stateMachine.state) {
            case ForwardCorrection:
                this.correctionSpeed = getCorrectionSpeed(true);
                SmartDashboard.putNumber("Balance Speed", this.correctionSpeed);
                mDrivetrain.moveRobotFrontBack(true, this.correctionSpeed);
                break;
            case ReverseCorrection:
                this.correctionSpeed = getCorrectionSpeed(false);
                SmartDashboard.putNumber("Balance Speed", this.correctionSpeed);
                mDrivetrain.moveRobotFrontBack(false, this.correctionSpeed);
                break;
            case CorrectionComplete:
                this.correctionsCompleted++;
                mDrivetrain.stopDrive();
                break;
            case Waiting:
            case MaybeBalanced:
            case Balanced:
                mDrivetrain.stopDrive();
                break;
            default:
                break;
        }
    }

    private double getCorrectionSpeed(boolean forward) {
        if (forward) {
            return DrivetrainConstants.balanceMoveSpeed * getCorrectionFactor();
        } else {
            return DrivetrainConstants.balanceMoveSpeed * getCorrectionFactor();
        }
    }

    private double getCorrectionFactor() {
        double correctionFactor = 1.0;
        if (this.correctionsCompleted > 0) {
            correctionFactor *= 0.8 * Math.max(0.3, 1 - 0.1 * this.correctionsCompleted);
        }
        SmartDashboard.putNumber("Balance Correction Factor", correctionFactor);
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
        public final int WAIT_CYCLES_IN_TOLERANCE = 250;
        public final int WAIT_CYCLES_NEXT_CORRECTION = 100;
        
        public enum BalanceModeState {
            Uncorrected,
            ForwardCorrection,
            ReverseCorrection,
            Waiting,
            Balanced,
            MaybeBalanced,
            CorrectionComplete
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
            SmartDashboard.putBoolean("Charge Station Moving", chargeStationMoving(currentPitchDelta));
            SmartDashboard.putBoolean("Charge Station Level", chargeStationLevel(currentPitch));
    
            switch (this.state) {
                case MaybeBalanced:
                    checkStillBalanced(currentPitch, currentPitchDelta);
                    break;
                case Uncorrected:
                    maybeCorrect(currentPitch, currentPitchDelta);
                    break;
                case ForwardCorrection:
                    resetBalancedCycleCounter();
                    maybeCompleteCorrection(currentPitch, currentPitchDelta);
                    break;
                case ReverseCorrection:
                    resetBalancedCycleCounter();
                    maybeCompleteCorrection(currentPitch, currentPitchDelta);
                    break;
                case Waiting:
                    waitCycle();
                    break;
                case CorrectionComplete:
                    waitForNextCorrection();
                    break;
                case Balanced:
                    break;
            }
        }

        public void resetBalancedCycleCounter() {
            this.balancedCycleCounter = 0;
        }

        public boolean canMakeCorrection() {
            return this.state == BalanceModeState.Uncorrected;
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
            return (currentPitchDelta < -pitchDeltaTolerance) && (currentPitchDelta > pitchDeltaTolerance);
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
            boolean moving = chargeStationMoving(currentPitchDelta);
            boolean level = chargeStationLevel(currentPitch);
            if (moving || level) {
                this.state = BalanceModeState.CorrectionComplete;
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
                this.state = BalanceModeState.MaybeBalanced;
            }
        }

        public void finish() {
            this.state = BalanceModeState.Balanced;
        }

        public void checkStillBalanced(double currentPitch, double currentPitchDelta) {
            if (!chargeStationLevel(currentPitch)) {
                this.state = BalanceModeState.Uncorrected;
            } else {
                this.balancedCycleCounter++;
                if (this.balancedCycleCounter > this.WAIT_CYCLES_IN_TOLERANCE) {
                    finish();
                }
            }
        }
    }
}
