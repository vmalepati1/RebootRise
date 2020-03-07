package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.indexer;

public class AlignToShoot extends CommandBase {

    @Override
    public void initialize() {
        if (indexer.isAlignedToShoot()) {
            end(true);
        }
    }

    @Override
    public void execute() {
        indexer.setIndexerMotorDutyCycle(0.35);
    }

    @Override
    public boolean isFinished() {
        return indexer.isAlignedToShoot();
    }

    @Override
    public void end(boolean interrupted) {
        indexer.setIndexerMotorDutyCycle(0);
    }
}
