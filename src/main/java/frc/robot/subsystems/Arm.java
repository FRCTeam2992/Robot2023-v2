// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private TalonFX armMotor;

  private CANCoder armEncoder;

  private int dashboardCounter;

  private double targetAngleDeg = 0;

  public enum EncoderState {
    UNKNOWN,
    CANCODER_FAILED,
    TOP_SLOP_ZONE,
    BOTTOM_SLOP_ZONE,
    CALIBRATED
  }

  private EncoderState motorEncoderConfidentCalibrated = EncoderState.UNKNOWN;

  // Variables for managing "hold position" to prevent backdrive
  private boolean holdPositionRecorded = false; // Have we logged the hold position yet
  private double holdPosition; // arm motor encoder clicks

  public enum ArmPosition {
    PARALLEL_TO_ELEVATOR(45.0),
    MOVEMENT_THRESHOLD_2(15.0),
    MOVEMENT_THRESHOLD_6(27.0),
    MOVEMENT_THRESHOLD_9(90.0);

    public final double positionDegrees;

    private ArmPosition(double positionDegrees) {
      this.positionDegrees = positionDegrees;
    }
  }

  public Arm() {
    armMotor = new TalonFX(Constants.ArmConstants.DeviceIDs.armMotor, "CanBus2");
    armMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    armMotor.setSensorPhase(false);
    armMotor.setInverted(TalonFXInvertType.Clockwise);
    // armMotor.setNeutralMode(NeutralMode.Brake);
    armMotor.setNeutralMode(NeutralMode.Brake);

    armEncoder = new CANCoder(Constants.ArmConstants.DeviceIDs.armEncoder, "CanBus2");
    armEncoder.configSensorDirection(false);
    armEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    // Wait for CANCoder config to take effect
    Timer.delay(1.0);

    initArmMotorEncoder();
    setPIDConstants();
  }

  @Override
  public void periodic() {
    if (dashboardCounter++ >= 5) {

      if (hasArmMotorReset()) {
          // initArmMotorEncoder();
      }

      SmartDashboard.putNumber("Arm CANcoder", getArmCANCoderPositionCorrected());
      SmartDashboard.putNumber("Arm Motor Encoder Raw", getArmMotorPositionRaw());

      SmartDashboard.putNumber("Arm Motor Encoder Degrees", getArmMotorPositionDeg());

      // SmartDashboard.putData("ZeroArm", new InstantCommand(() ->
      // initArmMotorEncoder()));

      dashboardCounter = 0;
    }
    // This method will be called once per scheduler run
  }

  public void setArmPosition(double angle) {
    holdPositionRecorded = false; // Hold position invalidated since we moved
    targetAngleDeg = angle;
    double position = angle * Constants.ArmConstants.motorEncoderClicksPerDegree;
    armMotor.set(ControlMode.MotionMagic, position);
  }

  public void holdArm() {
    if (!holdPositionRecorded) {
      // We haven't recorded where we are yet, so get it
      holdPosition = getArmMotorPositionRaw(); // encoder clicks
      holdPositionRecorded = true;
    } else {
      armMotor.set(ControlMode.MotionMagic, holdPosition);
    }

  }

  public double getArmMotorPositionRaw() {
      return armMotor.getSelectedSensorPosition();
  }

  public double getArmMotorPositionDeg() {
    return getArmMotorPositionRaw() / Constants.ArmConstants.motorEncoderClicksPerDegree;
  }

  public void setArmSpeed(double speed) {
    holdPositionRecorded = false; // Hold position invalidated since we moved
    if (getArmMotorPositionDeg() > Constants.ArmConstants.Limits.softStopTop) {
        speed = Math.min(0.05, speed);
    } else if (getArmMotorPositionDeg() < Constants.ArmConstants.Limits.softStopBottom) {
        speed = Math.max(-0.05, speed);
    }
    armMotor.set(ControlMode.PercentOutput, speed);
  }

  public double getArmCANCoderPositionRaw() {
    return armEncoder.getAbsolutePosition();
  }

  public double getArmCANCoderPositionCorrected() {
      double value = armEncoder.getAbsolutePosition() + Constants.ArmConstants.CANCoderOffset;
      if (value > 180.0) {
          value -= 360.0;
      } else if (value < -180.0) {
          value += 360.0;
      }
      return value;
  }

  public EncoderState motorEncoderCalibrated() {
    return motorEncoderConfidentCalibrated;
  }

  public void clearMotorEncoder() {
    motorEncoderConfidentCalibrated = EncoderState.UNKNOWN;
  }

  public boolean atPosition() {
    return (Math.abs(targetAngleDeg - getArmMotorPositionDeg()) < Constants.ArmConstants.armAngleToleranceDeg);
  }

  public void onDisable() {
    setArmSpeed(0.0);
  }

  public void initArmMotorEncoder() {
    double value = getArmCANCoderPositionCorrected();

    // if (!hasArmMotorReset() && motorEncoderConfidentCalibrated ==
    // EncoderState.CALIBRATED) {
    // // We previously had a good reset and no motor reset so still good
    // return;
    // }
    if (value >= Constants.ArmConstants.ArmSlopConstants.topZoneHiEdge) {
      // Above the top slop zone -- apply adjustment
      value -= Constants.ArmConstants.ArmSlopConstants.topZoneAdjustment;
      motorEncoderConfidentCalibrated = EncoderState.CALIBRATED;
    } else if (value >= Constants.ArmConstants.ArmSlopConstants.topZoneLowEdge) {
      // In the top slop zone -- assume midpoint but note we don't have good reading
      value -= (Constants.ArmConstants.ArmSlopConstants.topZoneAdjustment / 2.0);
      motorEncoderConfidentCalibrated = EncoderState.TOP_SLOP_ZONE;
      // System.out.println("++++++++++++> Arm init in top slop zone");
    } else {
        // Below the slop zone -- no adjustment needed
      motorEncoderConfidentCalibrated = EncoderState.CALIBRATED;
    }

    // Convert from degrees to encoder clicks
    // System.out.println("==========================================> Encoder set
    // to " + value);
    value *= Constants.ArmConstants.motorEncoderClicksPerDegree;
    // System.out.println("******************************** Target Arm Encoder
    // value: " + value);
    armMotor.setSelectedSensorPosition(value);
    // Timer.delay(1.0);
    // System.out.println("******************************** Arm encoder after set: "
    // + getArmMotorPositionRaw());

  }

  public boolean hasArmMotorReset() {
    return armMotor.hasResetOccurred();
  }

  public void setPIDConstants() {
    armMotor.config_kP(0, Constants.ArmConstants.PIDConstants.P);
    armMotor.config_kI(0, Constants.ArmConstants.PIDConstants.I);
    armMotor.config_kD(0, Constants.ArmConstants.PIDConstants.D);
    armMotor.config_kF(0, Constants.ArmConstants.PIDConstants.FF);

    armMotor.configMotionAcceleration(Constants.ArmConstants.PIDConstants.acceleration);
    armMotor.configMotionCruiseVelocity(Constants.ArmConstants.PIDConstants.cruiseVelocity);
  }

}
