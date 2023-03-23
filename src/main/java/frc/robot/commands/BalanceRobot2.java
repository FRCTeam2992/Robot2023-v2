// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class BalanceRobot2 extends CommandBase {
    private Drivetrain mDrivetrain;
    private LinearFilter lowPass;
    private double priorPitch;
    private double currentPitch;
    private double currentPitchDelta;
    private PIDController pitchPID;

    /** Creates a new BalanceRobot. */
    public BalanceRobot2(Drivetrain driveTrain) {
        // Use addRequirements() here to declare subsystem dependencies.
        mDrivetrain = driveTrain;
        addRequirements(mDrivetrain);
        lowPass = LinearFilter.movingAverage(3);
        pitchPID = new PIDController(.025, 0, 0.004);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        priorPitch = currentPitch = mDrivetrain.getRobotPitch();
        lowPass.reset();
        mDrivetrain.setDriveNeutralMode(NeutralMode.Brake);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        priorPitch = currentPitch;
        currentPitch = lowPass.calculate(mDrivetrain.getRobotPitch());
        currentPitchDelta = currentPitch - priorPitch;

        SmartDashboard.putNumber("Robot pitch lowpassed", currentPitch);
        SmartDashboard.putNumber("Robot pitchDelta", currentPitchDelta);

        double speed = pitchPID.calculate(currentPitch);
        speed = Math.max(-0.75, speed);
        speed = Math.min(0.75, speed);

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
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
