// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class BalanceRobot2 extends CommandBase {
    private Drivetrain mDrivetrain;
    private LinearFilter lowPass;
    private double priorPitch;
    private double currentPitch;
    private double currentPitchDelta;
    private PIDController pitchPID;
    private boolean reached = false;
    private Timer doneTimer;

    /** Creates a new BalanceRobot. */
    public BalanceRobot2(Drivetrain driveTrain) {
        // Use addRequirements() here to declare subsystem dependencies.
        mDrivetrain = driveTrain;
        addRequirements(mDrivetrain);
        lowPass = LinearFilter.movingAverage(3);
        pitchPID = new PIDController(Constants.DrivetrainConstants.balanceP,
                Constants.DrivetrainConstants.balanceI,
                Constants.DrivetrainConstants.balanceD);
        doneTimer = new Timer();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        priorPitch = currentPitch = mDrivetrain.getRobotPitch();
        lowPass.reset();
        mDrivetrain.setDriveNeutralMode(NeutralMode.Brake);
        reached = false;
        doneTimer.reset();
        doneTimer.stop();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double speed;

        priorPitch = currentPitch;
        currentPitch = lowPass.calculate(mDrivetrain.getRobotPitch());
        currentPitchDelta = currentPitch - priorPitch;

        SmartDashboard.putNumber("Robot pitch lowpassed", currentPitch);
        SmartDashboard.putNumber("Robot pitchDelta", currentPitchDelta);
        SmartDashboard.putBoolean("Robot Balance Reached", reached);

        speed = pitchPID.calculate(currentPitch);
        if ((Math.abs(currentPitch) < Constants.DrivetrainConstants.pitchTolerance) || reached) {
            reached = true;
            speed *= 0.3;
        }

        if (Math.abs(currentPitch) < Constants.DrivetrainConstants.pitchTolerance) {
            // We are within the balanced range -- start timer
            doneTimer.start();
        } else {
            // Outside the range so reset the timer
            doneTimer.restart();
        }
        speed = Math.max(-Constants.DrivetrainConstants.balanceMoveSpeed, speed);
        speed = Math.min(Constants.DrivetrainConstants.balanceMoveSpeed, speed);

        if (speed > 0.0) {
            mDrivetrain.moveRobotFrontBack(true, -speed);
        } else {
            mDrivetrain.moveRobotFrontBack(false, speed);
        }
        SmartDashboard.putNumber("Balance speed", speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        CommandScheduler.getInstance().schedule(mDrivetrain.XWheels().withTimeout(0.3));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return ((doneTimer.get() > Constants.DrivetrainConstants.balanceWaitTimer) ||
                (Robot.balanceTimer.get() > 14.0));
    }
}
