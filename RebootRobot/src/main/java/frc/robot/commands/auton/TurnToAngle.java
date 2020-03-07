package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.drivetrain;

public class TurnToAngle extends CommandBase {

    private double targetAngle;
    private int atTargetCount;

    private ProfiledPIDController controller = new ProfiledPIDController(0.013, 0, 0,
            new TrapezoidProfile.Constraints(400, 400)); //0.009

    public TurnToAngle(double targetAngle) {
        addRequirements(drivetrain);

        this.targetAngle = targetAngle;
    }

    @Override
    public void initialize() {
        System.out.println("turning to " + targetAngle);

        controller.reset(new TrapezoidProfile.State(0, 0));
        atTargetCount = 0;

        System.out.println("P CONSTANT: " + controller.getP());
        System.out.println("I CONSTANT: " + controller.getI());

        SmartDashboard.putNumber("Turn Setpoint", targetAngle);
    }

    @Override
    public void execute() {
        double turnRate = -controller.calculate(drivetrain.getHeading().getDegrees(), targetAngle);
        SmartDashboard.putNumber("Turn velocity error", controller.getVelocityError());
        SmartDashboard.putNumber("Turn position error", controller.getPositionError());
        System.out.println(controller.getPositionError());
        drivetrain.setDutyCycles(turnRate, -turnRate);

        if (controller.atSetpoint()) {
            atTargetCount++;
        } else {
            atTargetCount = 0;
        }

//        if (atTargetCount == 10) {
//            double tx = LimelightHelper.getTX();
//            double heading = drivetrain.getHeading().getDegrees();
//            System.out.println("OLD SETPOINT: " + heading);
//            System.out.println("NEW SETPOINT: " + (heading - tx));
//            currentRobot.getTurnPIDController().setSetpoint(heading - tx);
//            SmartDashboard.putNumber("Turn Setpoint", currentRobot.getTurnPIDController().getSetpoint());
//        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("AT TARGET COUNT: " + atTargetCount);
        drivetrain.setDutyCycles(0, 0);
    }

    @Override
    public boolean isFinished() {
        return controller.atGoal();
    }
}
