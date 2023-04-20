package frc.lib.manipulator;

import java.awt.geom.Point2D;

import frc.robot.subsystems.Elevator.ElevatorState;

public class Waypoint extends Point2D.Double {
    private OuttakeType outtake;
    private ElevatorState elevState;
    private double elevatorDelay; // How long to wait after elevator move to move tower

    public Waypoint(double height, double angle, OuttakeType outtake,
            ElevatorState elevState, double elevatorDelay) {
        super(height, angle);
        this.outtake = outtake;
        this.elevState = elevState;
        this.elevatorDelay = elevatorDelay;
    }

    public double height() {
        return this.x;
    }

    public double angle() {
        return this.y;
    }

    public OuttakeType outtakeType() {
        return outtake;
    }

    public ElevatorState elevatorState() {
        return elevState;
    }

    public double delay() {
        return elevatorDelay;
    }

    public static enum OuttakeType {
        Unknown(-0.6, 1.0), // Not a waypoint we outtake at or unknown so use some defaults
        Assumed_Cube(-0.7, 1.0),
        Assumed_Cone(-0.5, 1.0),
        Hi_Cone(-0.5, 1.0),
        Mid_Cone(-0.5, 1.0),
        Hi_Cube(-0.7, 1.0),
        Mid_Cube(-0.8, 1.0),
        Hybrid(-0.35, 1.0),
        Max_Throw_Cube(-1.0, 1.0),
        Rear_Low_Cube(-0.25, 1.0);

        public double speed; // How fast to spin claw
        public double time; // How long to spin claw

        private OuttakeType(double speed, double time) {
            this.speed = speed;
            this.time = time;
        }
    }

}
