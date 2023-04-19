package frc.lib.autonomous;

import java.util.Arrays;
import java.util.List;

public enum AutoSequence {
    Do_Nothing("Do Nothing",
            AutoStartPosition.LoadStationEnd,
            AutoStartPosition.CenterLoadStationSide,
            AutoStartPosition.CenterWallSide,
            AutoStartPosition.WallEnd),
    SideMobilityOnly("Side Mobility Only",
            AutoStartPosition.LoadStationEnd,
            AutoStartPosition.WallEnd),
    SideMobilityBalance("Side Mobility + Balance",
            AutoStartPosition.LoadStationEnd,
            AutoStartPosition.WallEnd),
    SideIntakeBalance("Side Intake + Balance",
            AutoStartPosition.LoadStationEnd,
            AutoStartPosition.WallEnd),
    Side2Scores("Side 2 Scores",
            AutoStartPosition.LoadStationEnd,
            AutoStartPosition.WallEnd),
    Side3Scores("Side 3 Scores",
            AutoStartPosition.LoadStationCube,
            AutoStartPosition.WallCube),
    CenterBalance("Center Cross + Balance",
            AutoStartPosition.CenterLoadStationSide,
            AutoStartPosition.CenterWallSide),
    CenterIntakeBalance("Center Intake Balance",
            AutoStartPosition.CenterLoadStationSide,
            AutoStartPosition.CenterWallSide),
    Center2ScoreBalance("Center 2 Score Balance",
            AutoStartPosition.CenterLoadStationSide,
            AutoStartPosition.CenterWallSide),
    Side2ScoreBalance("Side 2 Scores + Balance",
            AutoStartPosition.LoadStationEnd);
    public String description;
    public List<AutoStartPosition> allowedStartPositions;

    private AutoSequence(String description, AutoStartPosition... allowedStartPositions) {
        this.description = description;
        this.allowedStartPositions = Arrays.asList(allowedStartPositions);
    }
}