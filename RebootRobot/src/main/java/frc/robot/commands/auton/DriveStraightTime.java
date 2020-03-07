package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;
import static frc.robot.Robot.drivetrain;

public class DriveStraightTime extends CommandBase {

    private double power;
    private double timeS;
    private double startTime;

    public DriveStraightTime(double power, double timeS) {
        this.power = power;
        this.timeS = timeS;
    }

    @Override
    public void initialize() {
        startTime = getFPGATimestamp();
        drivetrain.setDutyCycles(0, 0);
    }

    @Override
    public void execute() {
        drivetrain.setDutyCycles(power, power);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setDutyCycles(0, 0);
    }

    @Override
    public boolean isFinished() {
        return getFPGATimestamp() - startTime >= timeS;
    }

}
