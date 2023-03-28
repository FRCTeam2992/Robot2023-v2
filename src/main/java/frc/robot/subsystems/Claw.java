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

    public Claw() {
        clawMotor = new TalonFX(Constants.ClawConstants.DeviceIDs.clawMotor, "CanBus2");
        clawMotor.setInverted(false);
        clawMotor.setNeutralMode(NeutralMode.Brake);
        clawMotor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
        clawMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
                true, 10,
                25.0, 1.0));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (dashboardCounter++ >= 5) {
            SmartDashboard.putBoolean("Claw Beam Break Triggered", getBeamBreakTriggered());
            dashboardCounter = 0;
        }
    }

    public boolean getBeamBreakTriggered() {
        return clawMotor.isFwdLimitSwitchClosed() == 1;
    }

    public void setClawSpeed(double speed) {
        clawMotor.set(TalonFXControlMode.PercentOutput, speed);
    }

    public double getClawCurrent() {
        return clawMotor.getSupplyCurrent();
    }

    public void holdClaw() {
        setClawSpeed(Constants.ClawConstants.holdPositionPower);
    }

    public void onDisable() {
        clawMotor.set(ControlMode.PercentOutput, 0.0);
    }
}
