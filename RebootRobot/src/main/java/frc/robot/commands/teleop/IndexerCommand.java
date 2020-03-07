package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;
import static frc.robot.OI.*;
import static frc.robot.Robot.*;

public class IndexerCommand extends CommandBase {

    private static double toggleStartTime;

    public IndexerCommand() {
        addRequirements(indexer);
        toggleStartTime = -1;
    }

    @Override
    public void execute() {
        if (!isAuto) {
            indexer.setIndexerMotorDutyCycle(gamepad.getLeftY());

            if (shooter.isReadyToShoot) {
                indexer.setShootToggle(true);

                toggleStartTime = Timer.getFPGATimestamp();
            }

            if (toggleStartTime > 0 && Timer.getFPGATimestamp() - toggleStartTime >= 0.5) {
                toggleStartTime = -1;
                indexer.setShootToggle(false);
            }
        }
    }

}
