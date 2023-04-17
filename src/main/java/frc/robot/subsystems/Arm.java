// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    /** Creates a new Arm. */
    private TalonFX armMotor;

    private CANCoder armEncoder;

    private int dashboardCounter;

    private double targetAngleDeg = 0;

    private PIDController armController;
    private LinearFilter lowPass;
    private boolean pidMode = false; // Keep track of if we are in PID control mode or manual mode

    public enum EncoderState {
        UNKNOWN,
        CANCODER_FAILED,
        TOP_SLOP_ZONE,
        BOTTOM_SLOP_ZONE,
        CALIBRATED
    }

    // Variables for managing "hold position" to prevent backdrive
    private boolean holdPositionRecorded = false; // Have we logged the hold position yet
    private double holdPosition; // arm motor encoder clicks

    public Arm() {
        armMotor = new TalonFX(Constants.ArmConstants.DeviceIDs.armMotor, "CanBus2");
        armMotor.setInverted(TalonFXInvertType.Clockwise);
        armMotor.setNeutralMode(NeutralMode.Brake);

        armEncoder = new CANCoder(Constants.ArmConstants.DeviceIDs.armEncoder, "CanBus2");
        armEncoder.configSensorDirection(false);
        armEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        lowPass = LinearFilter.movingAverage(5);

        armController = new PIDController(Constants.ArmConstants.PIDConstants.P,
                Constants.ArmConstants.PIDConstants.I,
                Constants.ArmConstants.PIDConstants.D, 0.02);
        armController.setTolerance(Constants.ArmConstants.armAngleToleranceDeg);
    }

    @Override
    public void periodic() {
        if (dashboardCounter++ >= 5) {
            SmartDashboard.putNumber("Arm CANcoder", getArmCANCoderPositionCorrected());
            if (Constants.debugDashboard) {
                SmartDashboard.putBoolean("Arm PID Enabled", pidMode);
                SmartDashboard.putNumber("Arm PID Target", targetAngleDeg);
                SmartDashboard.putBoolean("Arm Hold Position Recorded", holdPositionRecorded);
            }
    
            dashboardCounter = 0;
        }

        if (pidMode) {
            moveArmToTarget();
        }
        // This method will be called once per scheduler run
    }

    // Called once to set PID target angle
    public void setArmTarget(double angle) {
        targetAngleDeg = Math.min(angle, Constants.ArmConstants.Limits.hardStopTop);
        targetAngleDeg = Math.max(targetAngleDeg, Constants.ArmConstants.Limits.hardStopBottom);
        holdPositionRecorded = true;
        holdPosition = targetAngleDeg;
        pidMode = true;
        lowPass.reset();
        armController.reset();
        armController.setSetpoint(targetAngleDeg);
    }

    public void moveArmToTarget() {
        if (Math.abs(getArmCANCoderPositionCorrected() - targetAngleDeg) > 10.0) {
            armController.reset();
            armController.setSetpoint(targetAngleDeg);
        }
        double speed = armController.calculate(lowPass.calculate(getArmCANCoderPositionCorrected()));
        if (getArmCANCoderPositionCorrected() > 9.0 && getArmCANCoderPositionCorrected() < 30.0 &&
                armController.getSetpoint() > 19.0 && armController.getSetpoint() < 21) {
            speed = Math.min(0.1, speed);
            speed = Math.max(-0.1, speed);
        } else {
            speed = Math.min(1.0, speed);
            speed = Math.max(-1.0, speed);
        }
        if (Constants.debugDashboard) {
            SmartDashboard.putNumber("Arm PID speed", speed);
        }
        armMotor.set(TalonFXControlMode.PercentOutput, speed);
    }

    public void holdArm() {
        if (!holdPositionRecorded) {
            // We haven't recorded where we are yet, so get it
            holdPosition = getArmCANCoderPositionCorrected(); // encoder clicks
            holdPositionRecorded = true;
            pidMode = true;
            targetAngleDeg = holdPosition;
            armController.setSetpoint(targetAngleDeg);
        } else {
            // Work for this is done in periodic due to pidMode = true;
        }
    }

    public void setArmSpeed(double speed) {
        holdPositionRecorded = false; // Hold position invalidated since we moved
        if (getArmCANCoderPositionCorrected() > Constants.ArmConstants.Limits.softStopTop) {
            speed = Math.min(0.05, speed);
        } else if (getArmCANCoderPositionCorrected() < Constants.ArmConstants.Limits.softStopBottom) {
            speed = Math.max(-0.05, speed);
        }
        pidMode = false;
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

    public boolean atPosition() {
        // return (Math.abs(targetAngleDeg - getArmMotorPositionDeg()) <
        // Constants.ArmConstants.armAngleToleranceDeg);
        return (Math
                .abs(targetAngleDeg - getArmCANCoderPositionCorrected()) < Constants.ArmConstants.armAngleToleranceDeg);
    }

    public void setArmMotorNeutralMode(NeutralMode mode) {
        armMotor.setNeutralMode(mode);
    }

    public void onDisable() {
        pidMode = false;
        holdPositionRecorded = false;
        setArmSpeed(0.0);
        setArmMotorNeutralMode(NeutralMode.Brake);
    }
}
