
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ButterflyWheels extends SubsystemBase {

    private Solenoid butterflyWheelsSolenoid;

    public enum ButterflyWheelsState {
        Undeployed(false),
        Deployed(true);

        public final boolean solenoidSetting;

        private ButterflyWheelsState(boolean solenoidSetting) {
            this.solenoidSetting = solenoidSetting;
        }
    }

    /** Creates a new ButterflyWheels. */
    public ButterflyWheels() {
        butterflyWheelsSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,
                Constants.ButterflyWheelsConstants.DeviceIDs.ButterflyWheelsSolenoid);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void setButterflyWheelsState(ButterflyWheelsState state) {
        butterflyWheelsSolenoid.set(state.solenoidSetting);
    }

    public boolean getButterflyWheelsSolenoidState() {
        return butterflyWheelsSolenoid.get();
    }

    public void onDisable() {
        setButterflyWheelsState(ButterflyWheelsState.Undeployed);
    }
}
