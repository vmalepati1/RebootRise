package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.PneumaticIDs.SHOOTER_TOGGLE_ID;
import static frc.robot.OI.*;
import static frc.robot.Robot.indexer;

public class Indexer extends SubsystemBase {

    private final Solenoid shootToggle = new Solenoid(SHOOTER_TOGGLE_ID);
    private final DigitalInput ballLoadedSensor = new DigitalInput(10);
    private final DigitalInput hopperAlignedSensor = new DigitalInput(11);
    private final DigitalInput shooterAlignedSensor = new DigitalInput(12);
    private final CANSparkMax indexerMotor = new CANSparkMax(30, CANSparkMaxLowLevel.MotorType.kBrushed);

    public Indexer() {
        SmartDashboard.putBoolean("Aligned to Intake", false);
        SmartDashboard.putBoolean("Aligned to Shoot", false);
        SmartDashboard.putBoolean("Ball Present", false);

        indexerMotor.setInverted(true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Aligned to Intake", isAlignedToIntake());
        SmartDashboard.putBoolean("Aligned to Shoot", isAlignedToShoot());
        SmartDashboard.putBoolean("Ball Present", isBallLoaded());
        SmartDashboard.putBoolean("Shoot piston out", indexer.getShootToggle());
    }

    public void setShootToggle(boolean state) {
        shootToggle.set(state);
    }

    public boolean getShootToggle() {
        return shootToggle.get();
    }

    public boolean isAlignedToIntake() {
        return hopperAlignedSensor.get();
    }

    public boolean isAlignedToShoot() {
        return shooterAlignedSensor.get();
    }

    public boolean isBallLoaded() {
        return ballLoadedSensor.get();
    }

    public void setIndexerMotorDutyCycle(double dutyCycle) {
        indexerMotor.set(dutyCycle);
    }

}
