package frc.robot.commands.indexer;

import frc.robot.subsystems.Index;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Advance extends CommandBase {

    private final Index index;

    public enum Position {
        HOPPER, SHOOT
    }

    private final Position pos;

    private boolean stopCmd = false;

    public Advance(Index subsystem, Position pos) {
        this.index = subsystem;
        this.pos = pos;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
      }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (this.pos == Position.HOPPER) {
            if (index.atHopper()) {
                stop();
            } else {
                attemptSpin();
            }
        } else if (this.pos == Position.SHOOT) {
            if (index.atShoot()) {
                stop();
            } else {
               attemptSpin();
            }
        } else {
            stop();
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        index.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        switch (pos) {
            case HOPPER:
                if (index.atHopper()) {
                    stop();
                }
                break;

            case SHOOT:
                if (index.atShoot()) {
                    stop();
                }
                break;
        
            default:
                stop();
                break;
        }
        return stopCmd;
    }
    
    private void stop() {
        index.stop();
        stopCmd = true;
    }
    
    private void attemptSpin() {
        if(!index.spinCW()) {
            stop();
        }
    }
}