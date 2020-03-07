package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.indexer;
import static frc.robot.Robot.shooter;

public class ShootWhenReady extends CommandBase {

    private boolean done = false;

    @Override
    public void execute() {
        if (shooter.isReadyToShoot) {
            indexer.setShootToggle(true);
            done = true;
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
