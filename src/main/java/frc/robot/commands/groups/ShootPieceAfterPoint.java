// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveArmToPoint;
import frc.robot.commands.MoveClaw;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootPieceAfterPoint extends SequentialCommandGroup {
    /** Creates a new ShootPieceAfterPoint. */
    public ShootPieceAfterPoint(Arm arm, Claw claw) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new MoveArmToPoint(arm, 1.0, 15),
                new MoveClaw(claw, -1.0).withTimeout(1));
    }
}
