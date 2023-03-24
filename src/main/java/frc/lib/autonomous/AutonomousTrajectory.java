package frc.lib.autonomous;

import java.util.Arrays;
import java.util.List;
import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public enum AutonomousTrajectory {
    LoadStationMobility(PathPlanner.loadPathGroup("LoadStationMobility", new PathConstraints(.5, .5))),
    WallMobility(PathPlanner.loadPathGroup("WallMobility", new PathConstraints(.5, .5))),
    LoadStationMobilityIntake(PathPlanner.loadPathGroup("LoadStationMobilityIntake", 2.0, 1.5)),
    WallMobilityIntake(PathPlanner.loadPathGroup("WallMobilityIntake", 2.0, 1.5)),
    LoadStationMobilityBalance(PathPlanner.loadPathGroup("LoadStationMobilityBalance", new PathConstraints(2.9, 2.5))),
    WallMobilityBalance(PathPlanner.loadPathGroup("WallMobilityBalance", new PathConstraints(2.9, 2.5))),
    LoadStation2Scores(PathPlanner.loadPathGroup("LoadStation2Scores", 4.0, 3.0)),
    Wall2Scores(PathPlanner.loadPathGroup("Wall2Scores", 4.0, 3.0)),
    CenterBalanceLoadStationSide(
            PathPlanner.loadPathGroup("CenterBalanceLoadStationSide", new PathConstraints(2.8, 2.25))),
    // CenterBalanceWallSide(PathPlanner.loadPathGroup("CenterBalanceWallSide", new
    // PathConstraints(2.8, 2.25)));
    CenterBalanceWallSide(PathPlanner.loadPathGroup("PathGroupTest",
            new PathConstraints(2.0, 2.25),
            new PathConstraints(1.0, 1.5),
            new PathConstraints(3.5, 2.25),
            new PathConstraints(2.0, 2.25)));

    public List<PathPlannerTrajectory> trajectoryGroup;

    private AutonomousTrajectory(List<PathPlannerTrajectory> trajectoryGroup) {
        this.trajectoryGroup = trajectoryGroup;
    }
}
