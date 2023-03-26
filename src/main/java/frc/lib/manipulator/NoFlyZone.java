package frc.lib.manipulator;

import java.awt.geom.Rectangle2D;

public class NoFlyZone extends Rectangle2D.Double {
    public enum ZoneType {
        Always,
        ElevatorDeployed
    }

    private ZoneType mType;

    public NoFlyZone(double minHeight, double minAngle, double maxHeight, double maxAngle, ZoneType type) {
        super(minHeight, minAngle, maxHeight - minHeight, maxAngle - minAngle);
        mType = type;
    }

    public ZoneType getType() {
        return mType;
    }
}
