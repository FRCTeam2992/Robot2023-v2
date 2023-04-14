// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;

public class MoveArmToPoint extends CommandBase {
  /** Creates a new MoveArm. */
  private Arm mArm;
  private Claw mClaw;
  private Drivetrain mDrivetrain;

  private double mArmSpeed;
  private double mLaunchPoint;
  private double mEndPoint;

  private double mGyroAngle;

  private boolean firstPass = false;

  private boolean hasRunClaw = false;

  public MoveArmToPoint(Arm subsystem, Claw claw, Drivetrain drivetrain, double armSpeed, double launchPoint,
      double endPoint, double gyroAngle) {
    // Use addRequirements() here to delare subsystem dependencies.
    mArm = subsystem;
    mClaw = claw;
    mDrivetrain = drivetrain;

    mArmSpeed = armSpeed;
    mLaunchPoint = launchPoint;
    mEndPoint = endPoint;

    mGyroAngle = gyroAngle;

    addRequirements(mArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasRunClaw = false;
    mArm.setArmMotorNeutralMode(NeutralMode.Coast);

    firstPass = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!firstPass && mDrivetrain.getRobotPitch() > mGyroAngle) {
      firstPass = true;
    }

    if (firstPass) {
    mArm.setArmSpeed(mArmSpeed);
    if (mArmSpeed > 0.0) {
      if (!hasRunClaw && mArm.getArmCANCoderPositionCorrected() > mLaunchPoint) {
        CommandScheduler.getInstance().schedule(new MoveClaw(mClaw, -0.5).withTimeout(1));
      }
    } else if (mArmSpeed < 0.0) {
      if (!hasRunClaw && mArm.getArmCANCoderPositionCorrected() < mLaunchPoint) {
        CommandScheduler.getInstance().schedule(new MoveClaw(mClaw, -0.5).withTimeout(1));
      }
    }
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mArm.setArmSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (mArmSpeed > 0.0) {
      return mArm.getArmCANCoderPositionCorrected() > mEndPoint;
    } else if (mArmSpeed < 0.0) {
      return mArm.getArmCANCoderPositionCorrected() < mEndPoint;
    } else {
      return true;
    }
  }
}
