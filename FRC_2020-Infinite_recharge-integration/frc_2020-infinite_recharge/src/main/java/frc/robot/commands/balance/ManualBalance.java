package frc.robot.commands.balance;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.gamepads.F310;
import frc.robot.subsystems.Balance;

public class ManualBalance extends CommandBase {
    private final Balance balance;
    private final Joystick opJoy;

    public ManualBalance(Balance balance, Joystick opJoy) {
        this.balance = balance;
        this.opJoy = opJoy;
        addRequirements(balance);
    }

    @Override
    public void initialize() {
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        balance.drive(opJoy.getRawAxis(F310.LEFT_X));
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}