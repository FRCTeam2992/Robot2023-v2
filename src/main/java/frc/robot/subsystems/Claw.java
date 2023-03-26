// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
    /** Creates a new Claw. */
    private TalonFX clawMotor;

    private int dashboardCounter;

    private boolean holdPositionRecorded = false; // Have we logged the hold position yet
    private double holdPosition; // motor encoder clicks

    public Claw() {
        clawMotor = new TalonFX(Constants.ClawConstants.DeviceIDs.clawMotor, "CanBus2");
        clawMotor.setInverted(false);
        clawMotor.setNeutralMode(NeutralMode.Brake);
        clawMotor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
        clawMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
                true, 5.0,
                25.0, 0.1));

        setPIDConstants(clawMotor);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (dashboardCounter++ >= 5) {
            SmartDashboard.putNumber("Claw Position", getClawMotorPosition());
            SmartDashboard.putBoolean("Claw Beam Break Triggered", getBeamBreakTriggered());
            dashboardCounter = 0;
        }
    }

    public boolean getBeamBreakTriggered() {
        return clawMotor.isFwdLimitSwitchClosed() == 1;
    }

    public void setClawSpeed(double speed) {
        holdPositionRecorded = false;
        clawMotor.set(TalonFXControlMode.PercentOutput, speed);
    }

    public double getClawMotorPosition() {
        return clawMotor.getSensorCollection().getIntegratedSensorPosition();
    }

    public void setClawMotorPosition(double position) {
        clawMotor.set(TalonFXControlMode.Position, position);
    }

    public double getClawCurrent() {
        return clawMotor.getSupplyCurrent();
    }

    public void holdClaw() {
        if (!holdPositionRecorded) {
            // We haven't recorded where we are yet, so get it
            holdPosition = getClawMotorPosition();
            holdPositionRecorded = true;
            clawMotor.set(TalonFXControlMode.PercentOutput, 0.0);
        } else {
            clawMotor.set(TalonFXControlMode.Position, holdPosition);
        }

    }

    private void setPIDConstants(TalonFX motor) {
        motor.config_kP(0, Constants.ClawConstants.PIDConstants.P);
        motor.config_kI(0, Constants.ClawConstants.PIDConstants.I);
        motor.config_kD(0, Constants.ClawConstants.PIDConstants.D);
        motor.config_kF(0, Constants.ClawConstants.PIDConstants.FF);
        motor.configClosedLoopPeakOutput(0, 0.2);
    }
    public void onDisable() {
        clawMotor.set(ControlMode.PercentOutput, 0.0);
        holdPositionRecorded = false;
    }
}
