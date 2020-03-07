/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.indexer.Advance;
import frc.robot.commands.shooter.SpinUp;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Index;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SetupShot extends ParallelCommandGroup {
  /**
   * Creates a new SetupShot.
   */
  public SetupShot(Shooter shooter, Index index) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());super();
    addCommands(new SpinUp(shooter), new Advance(index, Advance.Position.SHOOT));
  }
}
