package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.CANBusIDs.*;
import static frc.robot.Robot.shooter;

public class Shooter extends SubsystemBase {

    public final CANSparkMax flywheelMaster = new CANSparkMax(FLYWHEEL_MASTER_ID, CANSparkMax.MotorType.kBrushless);
    public final CANSparkMax flywheelSlave = new CANSparkMax(FLYWHEEL_SLAVE_ID, CANSparkMax.MotorType.kBrushless);

    public static final double kFlywheelFF = 2.138 / 1000;

    public boolean autoShouldShoot = false;
    public boolean isReadyToShoot = false;

    public Shooter() {
        setupFlywheel();
    }

    public void setupFlywheel() {
//        flywheelMaster.restoreFactoryDefaults();
//        flywheelSlave.restoreFactoryDefaults();

        flywheelMaster.setInverted(true);

        flywheelMaster.setIdleMode(CANSparkMax.IdleMode.kBrake);
        flywheelSlave.setIdleMode(CANSparkMax.IdleMode.kBrake);

        flywheelSlave.follow(flywheelMaster, true);

        flywheelMaster.enableVoltageCompensation(10);

        flywheelMaster.getPIDController().setP(0.00035, 0);
        flywheelMaster.getPIDController().setI(0, 0);
        flywheelMaster.getPIDController().setD(0, 0);

        SmartDashboard.putNumber("Flywheel P", 0.00035);

//        flywheelMaster.burnFlash();
//        flywheelSlave.burnFlash();
    }

    public void setFlywheelDutyCycle(double dutyCycle) {
        flywheelMaster.set(dutyCycle);
    }

    public void setFlywheelOutput(double velocity, double feedforward, int sparkMaxPIDSlot) {
        shooter.flywheelMaster.getPIDController().setP(SmartDashboard.getNumber("Flywheel P", 0.00035));
        shooter.flywheelMaster.getPIDController().setI(0);
        shooter.flywheelMaster.getPIDController().setD(0);
        flywheelMaster.getPIDController().setReference(velocity, ControlType.kVelocity, sparkMaxPIDSlot, feedforward, CANPIDController.ArbFFUnits.kVoltage);
    }

    public double getFlywheelSpeed() {
        return flywheelMaster.getEncoder().getVelocity();
    }

    @Override
    public void periodic() {
//    setFlywheelOutput(TalonFXControlMode.Velocity, SmartDashboard.getNumber("Flywheel Speed", getFlywheelSpeed()));
//    setFlywheelOutput(TalonFXControlMode.Velocity, 16500);
//    setFlywheelOutput(TalonFXControlMode.PercentOutput, 0.8);
//    flywheelMaster.config_kP(0, SmartDashboard.getNumber("Flywheel P", 0));
//    flywheelMaster.config_kD(0, SmartDashboard.getNumber("Flywheel D", 0));
//    flywheelMaster.config_IntegralZone(0, (int)SmartDashboard.getNumber("Flywheel Izone", 150));
        SmartDashboard.putNumber("Flywheel Speed", getFlywheelSpeed());
//    SmartDashboard.putNumber("PID Slot")
//    flywheelMaster.set(ControlMode.PercentOutput, 0.8);
//    SmartDashboard.putNumber("Error Delta", errorDelta);
//    errorDelta = getClosedLoopFlywheelError() - previousError;
//    closedLoopErrorAverage.update(getClosedLoopFlywheelError());
//    previousError = getClosedLoopFlywheelError();
    }

}
