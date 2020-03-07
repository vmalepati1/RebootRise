package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Balance extends SubsystemBase {

  // ** Components **
  private final CANSparkMax balance;

  /**
   * Creates a new ExampleSubsystem.
   */
  public Balance() {
    balance = new CANSparkMax(Constants.BALANCE_MOTOR_CAN, MotorType.kBrushless);
    balance.set(0);
  }

  public void drive(double kMag) {
    balance.set(kMag);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}