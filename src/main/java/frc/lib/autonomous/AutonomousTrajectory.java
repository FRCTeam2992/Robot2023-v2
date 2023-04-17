package frc.lib.autonomous;

import java.util.List;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public enum AutonomousTrajectory {
    LoadStationMobility(PathPlanner.loadPathGroup("LoadStationMobility", new PathConstraints(.5, .5))),
    WallMobility(PathPlanner.loadPathGroup("WallMobility", new PathConstraints(.5, .5))),
    // LoadStationMobilityIntake(PathPlanner.loadPathGroup("LoadStationMobilityIntake",
    // 2.0, 1.5)),
    // WallMobilityIntake(PathPlanner.loadPathGroup("WallMobilityIntake", 2.0,
    // 1.5)),
    LoadStationMobilityBalance(PathPlanner.loadPathGroup("LoadStationMobilityBalance", new PathConstraints(2.9, 2.5))),
    WallMobilityBalance(PathPlanner.loadPathGroup("WallMobilityBalance", new PathConstraints(2.9, 2.5))),
    LoadStation2Scores(PathPlanner.loadPathGroup("LoadStation2Scores", 4.0, 1.8)),
    LoadStation2ScoreBalance(PathPlanner.loadPathGroup("LoadStation2ScoreBalance",
            new PathConstraints(4.0, 2.15),
            new PathConstraints(3.0, 2.5))),
    LoadStation3ScoresPart1(PathPlanner.loadPathGroup("LoadStation3Scores-Part1copy",
            new PathConstraints(4.5, 2.95))),
    LoadStation3ScoresPart2(PathPlanner.loadPathGroup("LoadStation3Scores-Part2copy",
            new PathConstraints(4.5, 2.95))),
    Wall2Scores(PathPlanner.loadPathGroup("Wall2Scores", 3.5, 2.0)),
    Wall2ScoreBalance(PathPlanner.loadPathGroup("Wall2ScoreBalance",
            new PathConstraints(4.0, 1.8),
            new PathConstraints(3.0, 2.5))),
    LoadStationIntakeBalance(PathPlanner.loadPathGroup("LoadStationIntakeBalance", 2.5, 1.8)),
    WallIntakeBalance(PathPlanner.loadPathGroup("WallIntakeBalance", 2.5, 1.8)),
    CenterBalanceLoadStationSide(PathPlanner.loadPathGroup("CenterBalanceLoadStationSide",
            new PathConstraints(3.0, 2.5),
            new PathConstraints(1.5, 2.5),
            new PathConstraints(1.3, 1.0),
            new PathConstraints(3.0, 2.5))),
    CenterBalanceWallSide(PathPlanner.loadPathGroup("CenterBalanceWallSide",
            new PathConstraints(3.0, 2.5),
            new PathConstraints(1.5, 2.5),
            new PathConstraints(1.3, 1.0),
            new PathConstraints(3.0, 2.5))),
    CenterIntakeBalanceLoadStationSide(PathPlanner.loadPathGroup("CenterIntakeBalanceLoadStationSide",
            new PathConstraints(3.0, 2.5),
            new PathConstraints(1.2, 2.5),
            new PathConstraints(1.5, 1.2),
            new PathConstraints(3.0, 2.5))),
    CenterIntakeBalanceWallSide(PathPlanner.loadPathGroup("CenterIntakeBalanceWallSide",
            new PathConstraints(3.0, 2.5),
            new PathConstraints(1.5, 2.5),
            new PathConstraints(1.3, 1.0),
            new PathConstraints(3.0, 2.5)));

    public List<PathPlannerTrajectory> trajectoryGroup;

    private AutonomousTrajectory(List<PathPlannerTrajectory> trajectoryGroup) {
        this.trajectoryGroup = trajectoryGroup;
    }
}
