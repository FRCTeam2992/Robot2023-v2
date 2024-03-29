// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private TalonFX elevatorMotorLead;
  private TalonFX elevatorMotorFollow;

  private Solenoid elevatorSolenoid;

  private int dashboardCounter = 0;

  private double targetHeightInch = 0;

  private Debouncer limit1Debounce;
  private Debouncer limit2Debounce;
  private boolean alreadyLimited = false; // Keep track of transitions and only limit on rise

  // Variables for managing "hold position" to prevent backdrive
  private boolean holdPositionRecorded = false; // Have we logged the hold position yet
  private double holdPosition; // lead motor encoder clicks
  private ElevatorState currentState = ElevatorState.Undeployed;

  public enum ElevatorState {
    Undeployed(false),
    Deployed(true);

    public final boolean solenoidSetting;

    private ElevatorState(boolean solenoidSetting) {
      this.solenoidSetting = solenoidSetting;
    }
  }

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotorLead = new TalonFX(Constants.ElevatorConstants.DeviceIDs.elevatorMotorLead, "CanBus2");
    elevatorMotorLead.setInverted(TalonFXInvertType.CounterClockwise);
    elevatorMotorLead.setNeutralMode(NeutralMode.Brake);

    elevatorMotorFollow = new TalonFX(Constants.ElevatorConstants.DeviceIDs.elevatorMotorFollow, "CanBus2");
    elevatorMotorFollow.setNeutralMode(NeutralMode.Brake);
    elevatorMotorFollow.set(TalonFXControlMode.Follower, elevatorMotorLead.getDeviceID());
    elevatorMotorFollow.setInverted(TalonFXInvertType.OpposeMaster);

    setPIDConstants(elevatorMotorLead);

    elevatorSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,
        Constants.ElevatorConstants.DeviceIDs.elevatorSolenoid);

    limit1Debounce = new Debouncer(0.05, DebounceType.kRising);
    limit2Debounce = new Debouncer(0.05, DebounceType.kRising);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (dashboardCounter++ >= 5) {
      if (Constants.debugDashboard) {
        SmartDashboard.putNumber("Lead Elevator Encoder", getLeadElevatorPostion());
        SmartDashboard.putNumber("Follow Elevator Encoder", getFollowElevatorPostion());
  
        SmartDashboard.putNumber("Elevator Inches", getElevatorInches());
        SmartDashboard.putBoolean("Elevator Deployed State", getElevatorState() == ElevatorState.Deployed);
      }

      dashboardCounter = 0;
    }

    if ((limit1Debounce.calculate(elevatorMotorLead.getSensorCollection().isRevLimitSwitchClosed() == 1)) ||
        (limit2Debounce.calculate(elevatorMotorFollow.getSensorCollection().isFwdLimitSwitchClosed() == 1))) {
      if (!alreadyLimited) {
        // New transition rise on limit switches
        alreadyLimited = true;
        zeroElevatorEncoders();
      } else {
        // Do nothing -- we already recorded the limit switch hit
      }
    } else {
      // Off of the limit switches so reset state
      alreadyLimited = false;
    }
  }

  public void configureMotorFollowing() {
    elevatorMotorFollow.set(TalonFXControlMode.Follower, elevatorMotorLead.getDeviceID());
  }

  public double getLeadElevatorPostion() {
    return elevatorMotorLead.getSensorCollection().getIntegratedSensorPosition();
  }

  public double getFollowElevatorPostion() {
    return elevatorMotorFollow.getSensorCollection().getIntegratedSensorPosition();
  }

  public double getElevatorInches() {
    return encoderClicksToInches(getLeadElevatorPostion());
  }

  public void setElevatorSpeed(double speed) {
    holdPositionRecorded = false; // Hold position invalidated since we moved
    if (getElevatorInches() < Constants.ElevatorConstants.Limits.softStopBottom) {
      speed = Math.max(-0.1, speed); // Allow down at slow speed if encoder out of sync
    } else if (getElevatorInches() > Constants.ElevatorConstants.Limits.softStopTop) {
      speed = Math.min(0.0, speed);
    }
    elevatorMotorLead.set(TalonFXControlMode.PercentOutput, speed);
  }

  public void setElevatorPosition(double inches) {
      holdPositionRecorded = true;

    if (inches < Constants.ElevatorConstants.Limits.softStopBottom) {
      inches = Constants.ElevatorConstants.Limits.softStopBottom;
    } else if (inches > Constants.ElevatorConstants.Limits.softStopTop) {
      inches = Constants.ElevatorConstants.Limits.softStopTop;
    }
    targetHeightInch = inches;
    holdPosition = targetHeightInch;
    elevatorMotorLead.set(TalonFXControlMode.MotionMagic, inchesToEncoderClicks(inches));
    // System.out.println("MOVING: " + inchesToEncoderClicks(inches));
  }

  public void holdElevator() {
    if (!holdPositionRecorded) {
      // We haven't recorded where we are yet, so get it
      holdPosition = getLeadElevatorPostion();
      holdPositionRecorded = true;
      elevatorMotorLead.set(TalonFXControlMode.PercentOutput, 0.0);
    } else {
        elevatorMotorLead.set(TalonFXControlMode.MotionMagic, holdPosition, DemandType.ArbitraryFeedForward, 0.05);
    }

  }

  public void setElevatorState(ElevatorState state) {
    elevatorSolenoid.set(state.solenoidSetting);
    currentState = state;
  }

  public ElevatorState getElevatorState() {
      return currentState;
  }


  public void toggleElevatorDeploy() {
      if (currentState == ElevatorState.Undeployed) {
          currentState = ElevatorState.Deployed;
      } else {
          currentState = ElevatorState.Undeployed;
      }
      setElevatorState(currentState);
  }

  public boolean getElevatorSolenoidState() {
    return elevatorSolenoid.get();
  }

  public void onDisable() {
    setElevatorState(ElevatorState.Undeployed);
    setElevatorSpeed(0.0);
  }

  private double encoderClicksToInches(double encoderClicks) {
    return encoderClicks / Constants.ElevatorConstants.encoderClicksPerInch;
  }

  private double inchesToEncoderClicks(double inches) {
    return inches * Constants.ElevatorConstants.encoderClicksPerInch;
  }

  public boolean atPosition() {
    return (Math.abs(targetHeightInch - getElevatorInches()) < Constants.ElevatorConstants.elevatorHeightToleranceInch);
  }

  public void zeroElevatorEncoders() {
    elevatorMotorLead.set(TalonFXControlMode.PercentOutput, 0.0);

    holdPositionRecorded = false;
    holdPosition = 0.0;
    elevatorMotorLead.getSensorCollection().setIntegratedSensorPosition(0.0, 100);
    elevatorMotorFollow.getSensorCollection().setIntegratedSensorPosition(0.0, 100);
  }

  private void setPIDConstants(TalonFX motor) {
    motor.config_kP(0, Constants.ElevatorConstants.PIDConstants.P);
    motor.config_kI(0, Constants.ElevatorConstants.PIDConstants.I);
    motor.config_kD(0, Constants.ElevatorConstants.PIDConstants.D);
    motor.config_kF(0, Constants.ElevatorConstants.PIDConstants.FF);

    motor.configMotionCruiseVelocity(Constants.ElevatorConstants.PIDConstants.cruiseVelocity);
    motor.configMotionAcceleration(Constants.ElevatorConstants.PIDConstants.acceleration);

  }
}
