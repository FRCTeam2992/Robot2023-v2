// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.autonomous.AutoBuilder;
import frc.lib.manipulator.Waypoint.OuttakeType;
import frc.robot.Constants.TowerConstants;
import frc.robot.RobotState.IntakeModeState;
import frc.robot.commands.BalanceRobotPID;
import frc.robot.commands.ClawOuttake;
import frc.robot.commands.DeployButterflyWheels;
import frc.robot.commands.DeployElevator;
import frc.robot.commands.DriveSticks;
import frc.robot.commands.HoldArm;
import frc.robot.commands.HoldClaw;
import frc.robot.commands.HoldElevator;
import frc.robot.commands.LEDsToDefaultColor;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveArmToPoint;
import frc.robot.commands.MoveClaw;
import frc.robot.commands.MoveTowerToScoringPosition;
import frc.robot.commands.SetSwerveAngle;
import frc.robot.commands.MoveElevator;

import frc.robot.commands.SetLEDsColor;
import frc.robot.commands.SetScoringTarget;
import frc.robot.commands.ToggleDeployElevator;
import frc.robot.commands.ToggleEndgameState;
import frc.robot.commands.ZeroElevatorEncoders;
import frc.robot.commands.groups.AutoDoubleLoadStationIntakeCone;
import frc.robot.commands.groups.AutoDoubleLoadStationIntakeCube;
import frc.robot.commands.groups.AutoGroundIntakeCube;
import frc.robot.commands.groups.AutoSingleLoadStationIntake;
import frc.robot.commands.groups.SafeDumbTowerToPosition;
import frc.robot.commands.groups.StopIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ButterflyWheels;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.testing.commands.TestArmPID;
import frc.robot.testing.commands.TestClawIntake;
import frc.robot.testing.commands.TestClawOuttake;
import frc.robot.testing.commands.TestElevatorPID;
import frc.robot.testing.commands.TestTowerSafeMove;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic
 * should actually be handled in the {@link Robot} periodic methods
 * (other than the scheduler calls). Instead, the structure of the
 * robot (including subsystems, commands, and trigger mappings) should
 * be declared here.
 */
public class RobotContainer {
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController controller0 = new CommandXboxController(0);
    private final CommandXboxController controller1 = new CommandXboxController(1);

    public final RobotState mRobotState;
    public final AutoBuilder mAutoBuilder;

    public final Drivetrain mDrivetrain;

    public final Elevator mElevator;
    public final Arm mArm;
    public final Claw mClaw;

    public final ButterflyWheels mButterflyWheels;

    public final LEDs mLEDs;

    public final PowerDistribution pdh;

    public DigitalInput networkToggleSwitch = new DigitalInput(
            Constants.RobotConstants.DeviceIDs.networkToggleSwitch);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        mRobotState = new RobotState();

        mDrivetrain = new Drivetrain(mRobotState);
        mDrivetrain.setDefaultCommand(new DriveSticks(mDrivetrain, mRobotState));

        mElevator = new Elevator();
        mElevator.setDefaultCommand(new HoldElevator(mElevator));

        mArm = new Arm();
        // mArm.setDefaultCommand(new StopArm(mArm));
        mArm.setDefaultCommand(new HoldArm(mArm));

        mClaw = new Claw();
        mClaw.setDefaultCommand(new HoldClaw(mClaw));
        // mClaw.setDefaultCommand(new StopClaw(mClaw));

        mButterflyWheels = new ButterflyWheels();

        mLEDs = new LEDs();
        mLEDs.setDefaultCommand(new LEDsToDefaultColor(mLEDs, mRobotState));

        mAutoBuilder = new AutoBuilder(mRobotState, mDrivetrain, mElevator, mArm,
                mClaw, mLEDs);

        // Setup the Auto Selectors
        mAutoBuilder.setupAutoSelector();

        // Add dashboard things
        addSubsystemsToDashboard();
        addRobotStateToDashboard();
        updateMatchStartChecksToDashboard();

        // Configure the trigger bindings
        configureShuffleboardBindings();
        configRealButtonBindings();
        // (new TestControllers()).configTestButtonBindings(this);

        pdh = new PowerDistribution(1, ModuleType.kRev);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
     * constructor with an arbitrary predicate, or via the named factories in
     * t * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link
     * edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
     */

    private void configRealButtonBindings() {
        /*
         * DO NOT PUT TEST BUTTONS IN THIS
         * ONLY REAL BUTTONS FOR COMPETITION
         */

        // -----------------------controller0-----------------------

        // ABXY
        // A = auto-align for scoring
        controller0.a().onTrue(new InstantCommand(() -> {
            mDrivetrain.setScoringMode(true);
        }));
        controller0.a().onFalse(new InstantCommand(() -> {
            mDrivetrain.setScoringMode(false);
        }));

        // B = intake from load station
        controller0.b().onTrue(
                new AutoSingleLoadStationIntake(mElevator, mArm, mClaw, mLEDs, mRobotState));
        controller0.b().onTrue(new InstantCommand(() -> {
            mDrivetrain.setLoadingMode(true);
        }));
        controller0.b().onFalse(new InstantCommand(() -> {
            mDrivetrain.setLoadingMode(false);
        }));

        // X = ground intake cube
        controller0.x().onTrue(
                new AutoGroundIntakeCube(mElevator, mArm, mClaw, mLEDs, mRobotState));// cubes
        controller0.x().onTrue(new SetLEDsColor(mLEDs, Constants.LEDColors.purple));

        // Y = double load station shelf intake --> Use cube/cone mode to select
        // waypoint
        controller0.y().onTrue(
                new ConditionalCommand(new AutoDoubleLoadStationIntakeCone(mElevator, mArm, mClaw, mLEDs, mRobotState),
                        new AutoDoubleLoadStationIntakeCube(mElevator, mArm, mClaw, mLEDs, mRobotState),
                        () -> mRobotState.intakeMode == IntakeModeState.Cone));

        // D-Pad
        controller0.povLeft().whileTrue(mDrivetrain.XWheels());// X the wheels

        controller0.povUp().onTrue(new SetLEDsColor(mLEDs, Constants.LEDColors.yellow));
        controller0.povUp()
                .onTrue(new InstantCommand(
                        () -> mRobotState.intakeMode = RobotState.IntakeModeState.Cone));
        controller0.povDown().onTrue(new SetLEDsColor(mLEDs, Constants.LEDColors.purple));
        controller0.povDown()
                .onTrue(new InstantCommand(
                        () -> mRobotState.intakeMode = RobotState.IntakeModeState.Cube));

        // Bumpers/Triggers
        controller0.leftBumper().onTrue(new InstantCommand(
                () -> {
                    mDrivetrain.setDoFieldOreint(false);
                }));// Disable Field Orient
        controller0.leftBumper().onFalse(new InstantCommand(
                () -> {
                    mDrivetrain.setDoFieldOreint(true);
                }));// Disable Field Orient

        controller0.rightBumper().onTrue(new InstantCommand(
                () -> {
                    mDrivetrain.setInSlowMode(true);
                })); // Slow Mode
        controller0.rightBumper().onFalse(new InstantCommand(
                () -> {
                    mDrivetrain.setInSlowMode(false);
                })); // Slow Mode

        controller0.leftTrigger(0.6)
                .onTrue(new MoveTowerToScoringPosition(mElevator, mArm, mRobotState));
        controller0.leftTrigger(0.6)
                .onFalse(new InstantCommand(() -> mRobotState.currentOuttakeType = OuttakeType.Unknown)
                        .andThen(new DeployElevator(mElevator, mArm, mRobotState, ElevatorState.Undeployed))
                        .andThen(new SafeDumbTowerToPosition(mElevator, mArm, mRobotState, TowerConstants.normal)));

        controller0.rightTrigger(0.6)
                .onTrue(new ClawOuttake(mClaw, mRobotState));

        controller0.rightStick().onTrue(new StopIntake(mElevator, mArm, mClaw, mRobotState));
        controller0.rightStick().onTrue(new HoldClaw(mClaw));

        // Back and Start

        controller0.start().onTrue(new ResetGyro(mDrivetrain));

        controller0.back().onTrue(new BalanceRobotPID(mDrivetrain));

        // Joysticks Buttons

        // -----------------------controller1-----------------------
        // ABXY
        controller1.y().whileTrue(new MoveClaw(mClaw, Constants.ClawConstants.Intake.Speed.cone));

        // Bumpers/Triggers
        controller1.leftBumper().onTrue(new SetLEDsColor(mLEDs, Constants.LEDColors.purple));
        controller1.leftBumper()
                .onTrue(new InstantCommand(
                        () -> mRobotState.intakeMode = RobotState.IntakeModeState.Cube));
        controller1.rightBumper().onTrue(new SetLEDsColor(mLEDs, Constants.LEDColors.yellow));
        controller1.rightBumper()
                .onTrue(new InstantCommand(
                        () -> mRobotState.intakeMode = RobotState.IntakeModeState.Cone));

        controller1.leftTrigger(0.6).onTrue(new StopIntake(mElevator, mArm, mClaw, mRobotState));
        controller1.leftTrigger(0.6).onTrue(new HoldClaw(mClaw));
        controller1.rightTrigger(0.6)
                .onTrue(new SetScoringTarget(mRobotState, controller0, controller1, mElevator, mArm));

        // Back and Start
        controller1.start().onTrue(new ToggleEndgameState(mRobotState, mLEDs));
        controller1.back().onTrue(new DeployButterflyWheels(mButterflyWheels)
                .unless(() -> !mRobotState.isInEndgameMode()));

        // Joysticks and Buttons
        controller1.axisLessThan(XboxController.Axis.kLeftY.value, -0.6).whileTrue(
                new MoveArm(mArm, -0.40));
        controller1.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.6).whileTrue(
                new MoveArm(mArm, 0.40));

        controller1.axisLessThan(XboxController.Axis.kRightY.value, -0.6).whileTrue(
                new MoveElevator(mElevator, 0.4));
        controller1.axisGreaterThan(XboxController.Axis.kRightY.value, 0.6).whileTrue(
                new MoveElevator(mElevator, -0.4));
                
        controller1.leftStick().onTrue(new SetScoringTarget(mRobotState, controller0, controller1, mElevator, mArm));
        controller1.rightStick().onTrue(new ToggleDeployElevator(mElevator));

    }

    private void configureShuffleboardBindings() {
            SmartDashboard.putData("Arm To Point 100%, 10°",
                            new MoveArmToPoint(mArm, mClaw, mDrivetrain, 1.0, 10.0, 30.0, -5.0));
        if (Constants.debugDashboard) {
            SmartDashboard.putData("Scoring", new DeployElevator(mElevator, mArm, mRobotState, ElevatorState.Undeployed));
            SmartDashboard.putData("Loading", new DeployElevator(mElevator, mArm, mRobotState, ElevatorState.Deployed));

            SmartDashboard.putData("Move Elevator Down", new MoveElevator(mElevator, -0.1));
            SmartDashboard.putData("Stop Elevator", new MoveElevator(mElevator, 0.0));
            SmartDashboard.putData("Move Elevator Up", new MoveElevator(mElevator, 0.1));
            
            // SmartDashboard.putData("Re-init Arm Encoder", new InstantCommand(() ->
            // mArm.initArmMotorEncoder()));

            // SmartDashboard.putData("Intake Game Piece", new IntakeGamePiece(mClaw,
            // mRobotState));

            SmartDashboard.putNumber("Test Claw Cube In Spd %", 0.5);
            SmartDashboard.putNumber("Test Claw Cone In Spd %", 0.7);
            SmartDashboard.putData("Test Claw Intake", new TestClawIntake(mClaw, mRobotState));
    
            SmartDashboard.putNumber("Test Claw Cube Out Spd %", 0.7);
            SmartDashboard.putNumber("Test Claw Cone Out Spd %", 0.5);
            SmartDashboard.putData("Test Claw Outtake", new TestClawOuttake(mClaw, mRobotState));
    
            // SmartDashboard.putData("Reset Odometry to Red Inner Cone",
            // new InstantCommand(() -> mDrivetrain
            // .resetOdometryToPose(new Pose2d(1.89, 3.0307,
            // Rotation2d.fromDegrees(0.0)))));
            SmartDashboard.putData("0 Wheels", new SetSwerveAngle(mDrivetrain, 0, 0, 0, 0));

            // SmartDashboard.putData("Test Path Planner Path",
            // new FollowTrajectoryCommand(mDrivetrain, mDrivetrain.testPath, true));

            // SmartDashboard.putData("Deploy Butterfly Wheels", new
            // DeployButterflyWheels(mButterflyWheels));
            // SmartDashboard.putData("Test Path Planner Path",
            // new FollowTrajectoryCommand(mDrivetrain, mDrivetrain.testPath, true));

            SmartDashboard.putNumber("ElevTestMoveHeight", 20.0);
            SmartDashboard.putNumber("ArmTestMoveAngle", 0.0);
            SmartDashboard.putData("TestSafeDumbPath", new TestTowerSafeMove(mElevator,
                    mArm, mRobotState));
            SmartDashboard.putData("Test PID Move Arm", new TestArmPID(mArm));
            SmartDashboard.putData("Test PID Move Elevator", new TestElevatorPID(mElevator));

            // SmartDashboard.putData("TestAutoBalance", new BalanceRobot(mDrivetrain));
        }
        SmartDashboard.putData("Zero Elevator Encoder", new ZeroElevatorEncoders(mElevator));

        SmartDashboard.putData("Reset Odometry", mDrivetrain.ResetOdometry());
    }

    public void addSubsystemsToDashboard() {
        SmartDashboard.putData("Drivetrain", mDrivetrain);
        SmartDashboard.putData("Arm", mArm);
        SmartDashboard.putData("Claw", mClaw);
        SmartDashboard.putData("Elevator", mElevator);
        SmartDashboard.putData("Butterfly Wheels", mButterflyWheels);
        SmartDashboard.putData("LEDs", mLEDs);
    }

    public void addRobotStateToDashboard() {
        SmartDashboard.putBoolean("Target: Left Grid High Left",
                mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverLeft &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.HighLeft);
        SmartDashboard.putBoolean("Target: Left Grid High Center",
                mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverLeft &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.HighCenter);
        SmartDashboard.putBoolean("Target: Left Grid High Right",
                mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverLeft &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.HighRight);
        SmartDashboard.putBoolean("Target: Left Grid Mid Left",
                mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverLeft &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.MidLeft);
        SmartDashboard.putBoolean("Target: Left Grid Mid Center",
                mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverLeft &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.MidCenter);
        SmartDashboard.putBoolean("Target: Left Grid Mid Right",
                mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverLeft &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.MidRight);
        SmartDashboard.putBoolean("Target: Left Grid Low Left",
                mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverLeft &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.LowLeft);
        SmartDashboard.putBoolean("Target: Left Grid Low Center",
                mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverLeft &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.LowCenter);
        SmartDashboard.putBoolean("Target: Left Grid Low Right",
                mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverLeft &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.LowRight);

        SmartDashboard.putBoolean("Target: Center Grid High Left",
                mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridCenter &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.HighLeft);
        SmartDashboard.putBoolean("Target: Center Grid High Center",
                mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridCenter &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.HighCenter);
        SmartDashboard.putBoolean("Target: Center Grid High Right",
                mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridCenter &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.HighRight);
        SmartDashboard.putBoolean("Target: Center Grid Mid Left",
                mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridCenter &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.MidLeft);
        SmartDashboard.putBoolean("Target: Center Grid Mid Center",
                mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridCenter &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.MidCenter);
        SmartDashboard.putBoolean("Target: Center Grid Mid Right",
                mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridCenter &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.MidRight);
        SmartDashboard.putBoolean("Target: Center Grid Low Left",
                mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridCenter &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.LowLeft);
        SmartDashboard.putBoolean("Target: Center Grid Low Center",
                mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridCenter &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.LowCenter);
        SmartDashboard.putBoolean("Target: Center Grid Low Right",
                mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridCenter &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.LowRight);

        SmartDashboard.putBoolean("Target: Right Grid High Left",
                mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverRight &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.HighLeft);
        SmartDashboard.putBoolean("Target: Right Grid High Center",
                mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverRight &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.HighCenter);
        SmartDashboard.putBoolean("Target: Right Grid High Right",
                mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverRight &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.HighRight);
        SmartDashboard.putBoolean("Target: Right Grid Mid Left",
                mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverRight &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.MidLeft);
        SmartDashboard.putBoolean("Target: Right Grid Mid Center",
                mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverRight &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.MidCenter);
        SmartDashboard.putBoolean("Target: Right Grid Mid Right",
                mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverRight &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.MidRight);
        SmartDashboard.putBoolean("Target: Right Grid Low Left",
                mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverRight &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.LowLeft);
        SmartDashboard.putBoolean("Target: Right Grid Low Center",
                mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverRight &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.LowCenter);
        SmartDashboard.putBoolean("Target: Right Grid Low Right",
                mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverRight &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.LowRight);

        if (Constants.debugDashboard) {
            SmartDashboard.putBoolean("Blue Alliance",
                    DriverStation.getAlliance() == DriverStation.Alliance.Blue);
            SmartDashboard.putBoolean("Red Alliance",
                    DriverStation.getAlliance() == DriverStation.Alliance.Red);

            SmartDashboard.putBoolean("Endgame Mode",
                    mRobotState.endgameMode == RobotState.EndgameModeState.InEndgame);
        }
    }

    public void updateMatchStartChecksToDashboard() {
            if (mAutoBuilder.getAutoStartPosition() != null) {
        SmartDashboard.putString("Confirmed Auto Start Position",
                mAutoBuilder.getAutoStartPosition().description);
}
if (mAutoBuilder.getAutoSequence() != null && mAutoBuilder.autoStartCompatible()) {
        SmartDashboard.putString("Confirmed Auto Sequence", mAutoBuilder.getAutoSequence().description);
        } else {
            SmartDashboard.putString("Confirmed Auto Sequence", "INVALID SEQUENCE FOR THIS START POSN");
        }
        if (mAutoBuilder.getAutoPreloadScore() != null) {
        SmartDashboard.putString("Confirmed Auto Preload Score",
                mAutoBuilder.getAutoPreloadScore().description);
}
        SmartDashboard.putBoolean("Valid Auto Sequence?", mAutoBuilder.autoStartCompatible());
        SmartDashboard.putBoolean("Elevator Encoder Good?", Math.abs(mElevator.getElevatorInches()) <= 0.2);
    }

    public CommandXboxController getController0() {
        return controller0;
    }
}
