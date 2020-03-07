package frc.robot.commands.teleop;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Shooter;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;
import static frc.robot.OI.barfButton;
import static frc.robot.OI.shootButton;
import static frc.robot.Robot.shooter;

public class ShooterCommand extends CommandBase {

    private FlywheelState currentFlywheelState, previousFlywheelState;
    protected static double targetSpeed = 0;

    private static final double kStabilizationTime = 0.5;
    private static final double BARF_SPEED = 1000;
    private static final double BASELINE_SPEED = 4100;
    private static final int SPEED_ERROR_LIMIT = 200;

    private static double spinUpStartTime;

    public ShooterCommand() {
        addRequirements(shooter);

        SmartDashboard.putNumber("Flywheel Output", BASELINE_SPEED);

        currentFlywheelState = previousFlywheelState = FlywheelState.OFF;
    }


    @Override
    public void execute() {
//        double output = SmartDashboard.getNumber("Flywheel Output", 1000);
//
//        shooter.flywheelMaster.getPIDController().setP(SmartDashboard.getNumber("Flywheel P", 0.00035));
//        shooter.flywheelMaster.getPIDController().setI(0);
//        shooter.flywheelMaster.getPIDController().setD(0);
//
//        shooter.setFlywheelOutput(output, Shooter.kFlywheelFF * output, 0);
//        shooter.setFlywheelDutyCycle(0.5);
        SmartDashboard.putNumber("Target speed", targetSpeed);
        SmartDashboard.putNumber("Flywheel closed loop error", targetSpeed - shooter.getFlywheelSpeed());
        if (previousFlywheelState != currentFlywheelState) {
            currentFlywheelState.initialize();
        }
        previousFlywheelState = currentFlywheelState;

        // Run the states' execute methods, and update the pointers, as necessary.
        currentFlywheelState = currentFlywheelState.execute();

        SmartDashboard.putString("Shooter State", currentFlywheelState.name());
    }

    private enum FlywheelState {
        OFF {
            @Override
            public void initialize() {
                targetSpeed = 0;
            }

            @Override
            public FlywheelState execute() {
                if (shootButton.get() || (Robot.isAuto && shooter.autoShouldShoot)) {
                    // If shooting, estimate the target speed
                    targetSpeed = SmartDashboard.getNumber("Flywheel Output", BASELINE_SPEED);
                    return SPINNING_UP;
                }
                if (barfButton.get()) {
                    targetSpeed = BARF_SPEED;
                    return SPINNING_UP;
                }
                return this;
            }
        },
        SPINNING_UP {
            @Override
            public void initialize() {
                shooter.setFlywheelOutput(targetSpeed, targetSpeed * Shooter.kFlywheelFF, 0);
                shooter.isReadyToShoot = false;
                spinUpStartTime = Timer.getFPGATimestamp();
            }

            @Override
            public FlywheelState execute() {
                shooter.setFlywheelOutput(targetSpeed, targetSpeed * Shooter.kFlywheelFF, 0);

                if ((!Robot.isAuto && !(shootButton.get() || barfButton.get())) ||
                        (Robot.isAuto && !shooter.autoShouldShoot)) {
                    return SPINNING_DOWN;
                }

                double closedLoopFlywheelError = targetSpeed - shooter.getFlywheelSpeed();

                if (Timer.getFPGATimestamp() - spinUpStartTime > kStabilizationTime && (closedLoopFlywheelError < 0 ||
                        Math.abs(closedLoopFlywheelError) < SPEED_ERROR_LIMIT)) {
                    return SHOOTING;
                }
                return this;
            }
        },
        SHOOTING {
            @Override
            public void initialize() {
                shooter.setFlywheelOutput(targetSpeed, targetSpeed * Shooter.kFlywheelFF, 0);
                shooter.isReadyToShoot = true;
//        System.out.println("Shooting, error: " + turretShooter.getClosedLoopFlywheelError());

            }

            @Override
            public FlywheelState execute() {
//        intakeConveyor.resetBallCount();
//        SmartDashboard.putNumber("Delay Time", getFPGATimestamp() - delayStart);
                shooter.setFlywheelOutput(targetSpeed, targetSpeed * Shooter.kFlywheelFF, 0);
                if ((!Robot.isAuto && !(shootButton.get() || barfButton.get())) ||
                        (Robot.isAuto && !shooter.autoShouldShoot)) {
                    return SPINNING_DOWN;
                }

                double closedLoopFlywheelError = targetSpeed - shooter.getFlywheelSpeed();

                if (closedLoopFlywheelError >= 0 &&
                        Math.abs(closedLoopFlywheelError) > SPEED_ERROR_LIMIT) {
                    return SPINNING_UP;
                }
                return this;
            }
        },
        SPINNING_DOWN {
            @Override
            public void initialize() {
                shooter.isReadyToShoot = false;
                targetSpeed = 0;
            }

            @Override
            public FlywheelState execute() {
                shooter.setFlywheelDutyCycle(0);
                if (shootButton.get() || (Robot.isAuto && shooter.autoShouldShoot)) {
                    // If shooting, estimate the target speed
                    targetSpeed = SmartDashboard.getNumber("Flywheel Output", BASELINE_SPEED);
                    return SPINNING_UP;
                }
                if (barfButton.get()) {
                    targetSpeed = BARF_SPEED;
                    return SPINNING_UP;
                }
                if (Math.abs(shooter.getFlywheelSpeed()) < SPEED_ERROR_LIMIT) {
                    return OFF;
                }
                return this;
            }
        };

        public abstract void initialize();

        public abstract FlywheelState execute();
    }

}
