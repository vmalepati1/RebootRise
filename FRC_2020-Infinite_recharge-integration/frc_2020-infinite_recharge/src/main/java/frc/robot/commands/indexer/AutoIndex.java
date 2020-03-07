/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Index;

public class AutoIndex extends CommandBase {
  private final Index index;
  private boolean stopCmd = false;

  private enum IndexState {
    Init, Rotating, Waiting, End
  }
  private IndexState state = IndexState.Init;
  /**
   * Creates a new AutoIndex.
   */
  public AutoIndex(Index index) {
    this.index = index;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(index);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    index.stop();
    index.retractLoader();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (state) {
      case Init:
        if (index.canRotate()) {
          if (index.atHopper()) {
            index.stop();
            state = IndexState.Waiting;
          } else {
            index.spinCW();
            state = IndexState.Rotating;
          }
        } else {
          state = IndexState.Init;
        }
        break;
      case Rotating:
        if (index.atHopper()) {
          index.stop();
          state = IndexState.Waiting;
        } else {
          index.spinCW();
          state = IndexState.Rotating;
        }
        break;
      case Waiting:
        if (index.ballPresent()) {
          state = IndexState.End;
          index.stop();
        } else {
          index.stop();
          state = IndexState.Waiting;
        }
        break;
      case End:
        stopCmd = true;
        index.stop();
        break;
      default:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    index.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stopCmd;
  }
}
