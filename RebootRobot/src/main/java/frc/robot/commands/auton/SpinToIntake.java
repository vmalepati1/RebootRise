package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.EnhancedBoolean;

import static frc.robot.Robot.indexer;

public class SpinToIntake extends CommandBase {

    private boolean risingEdge;
    private EnhancedBoolean shooterSensorStatus;

    public SpinToIntake(boolean risingEdge) {
        this.risingEdge = risingEdge;
    }

    @Override
    public void initialize() {
        shooterSensorStatus = new EnhancedBoolean(indexer.isAlignedToIntake());
    }

    @Override
    public void execute() {
        shooterSensorStatus.set(indexer.isAlignedToIntake());
        indexer.setIndexerMotorDutyCycle(0.35);
    }

    @Override
    public boolean isFinished() {
        if (risingEdge) {
            return shooterSensorStatus.isRisingEdge() || indexer.getShootToggle();
        }
        return shooterSensorStatus.isFallingEdge() || indexer.getShootToggle();
    }

    @Override
    public void end(boolean interrupted) {
        indexer.setIndexerMotorDutyCycle(0);
    }

}
