// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  private TalonFX clawMotor;

  private int dashboardCounter;
  public Claw() {
    clawMotor = new TalonFX(Constants.ClawConstants.DeviceIDs.clawMotor);
    clawMotor.setInverted(false);
    clawMotor.setNeutralMode(NeutralMode.Brake);

    clawMotor.config_kP(0, Constants.ClawConstants.PIDConstants.P);
    clawMotor.config_kI(0, Constants.ClawConstants.PIDConstants.I);
    clawMotor.config_kD(0, Constants.ClawConstants.PIDConstants.D);
    clawMotor.config_kF(0, Constants.ClawConstants.PIDConstants.FF);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (dashboardCounter++ >= 5) {
      SmartDashboard.putNumber("Claw Position", getClawMotorPosition());
      dashboardCounter = 0;
    }
  }

  public void setClawSpeed(double speed) {
    clawMotor.set(ControlMode.PercentOutput, speed);
  }

  public double getClawMotorPosition() {
    return clawMotor.getSensorCollection().getIntegratedSensorPosition();
  }

  public void setClawMotorPosition(double position) {
    clawMotor.set(ControlMode.Position, position);
  }

  public double getClawCurrent() {
    return clawMotor.getSupplyCurrent();
  }

  public void onDisable() {
    clawMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
