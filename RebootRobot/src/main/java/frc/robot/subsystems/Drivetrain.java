package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.CANBusIDs.*;
import static frc.robot.Constants.RamseteConstants.TRACK_WIDTH;
import static frc.robot.Robot.drivetrain;

public class Drivetrain extends SubsystemBase {

    private final CANSparkMax rightWheelsMaster = new CANSparkMax(DRIVE_RIGHT_MASTER_ID, CANSparkMax.MotorType.kBrushless);
    private final CANSparkMax rightWheelsSlave = new CANSparkMax(DRIVE_RIGHT_SLAVE_ID, CANSparkMax.MotorType.kBrushless);
    private final CANSparkMax leftWheelsMaster = new CANSparkMax(DRIVE_LEFT_MASTER_ID, CANSparkMax.MotorType.kBrushless);
    private final CANSparkMax leftWheelsSlave = new CANSparkMax(DRIVE_LEFT_SLAVE_ID, CANSparkMax.MotorType.kBrushless);
    private DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(TRACK_WIDTH);
    private DifferentialDriveOdometry driveOdometry = new DifferentialDriveOdometry(getHeading());

    private RamseteController ramseteController = new RamseteController();

    private Pose2d robotPose = new Pose2d();

    private static final double kEncoderPositionFactor = (1 / 10.75) * Units.inchesToMeters(6) * Math.PI;;

    private static double previousHeading;
    private static double currentHeading;

    private static double previousDistanceLeft;
    private static double previousDistanceRight;

    private final SimpleMotorFeedforward drivetrainFeedforward = new SimpleMotorFeedforward(0.165, 2.04,0);

    private final PIDController leftPIDController = new PIDController(2.37, 0, 0);
    private final PIDController rightPIDController = new PIDController(2.37, 0, 0);

    public Drivetrain() {
        motorSetUp();
        resetHardware();

        previousHeading = 0;
        currentHeading = 0;

        previousDistanceLeft = 0;
        previousDistanceRight = 0;
    }

    @Override
    public void periodic() {
        updateRobotPose();

        SmartDashboard.putNumber("Angle", getHeading().getDegrees());
        SmartDashboard.putNumber("Left neo encoder velocity", drivetrain.getCANEncoderLeftVelocity());
        SmartDashboard.putNumber("right neo encoder velocity", drivetrain.getCANEncoderRightVelocity());
        SmartDashboard.putNumber("Left neo encoder distance", drivetrain.getCANEncoderLeftMeters());
        SmartDashboard.putNumber("right neo encoder distance", drivetrain.getCANEncoderRightMeters());

        double currentDistanceLeft = getCANEncoderLeftMeters();
        double currentDistanceRight = getCANEncoderRightMeters();
        double dLeft = currentDistanceLeft - previousDistanceLeft;
        double dRight = currentDistanceRight - previousDistanceRight;
        double theta = (dRight - dLeft) / TRACK_WIDTH;

        currentHeading = previousHeading + theta;

        previousHeading = currentHeading;
        previousDistanceLeft = currentDistanceLeft;
        previousDistanceRight = currentDistanceRight;
    }

    public void motorSetUp() {
//        leftWheelsMaster.restoreFactoryDefaults();
//        leftWheelsSlave.restoreFactoryDefaults();
//        rightWheelsMaster.restoreFactoryDefaults();
//        rightWheelsSlave.restoreFactoryDefaults();

        leftWheelsMaster.setInverted(false);
        rightWheelsMaster.setInverted(true);

        leftWheelsSlave.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftWheelsMaster.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightWheelsSlave.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightWheelsMaster.setIdleMode(CANSparkMax.IdleMode.kBrake);

        leftWheelsSlave.follow(leftWheelsMaster);
        rightWheelsSlave.follow(rightWheelsMaster);

        leftWheelsMaster.setOpenLoopRampRate(0);
        leftWheelsSlave.setOpenLoopRampRate(0);
        rightWheelsMaster.setOpenLoopRampRate(0);
        rightWheelsSlave.setOpenLoopRampRate(0);

        leftWheelsMaster.setSmartCurrentLimit(80);
        leftWheelsSlave.setSmartCurrentLimit(80);
        rightWheelsMaster.setSmartCurrentLimit(80);
        rightWheelsSlave.setSmartCurrentLimit(80);

        leftWheelsMaster.getEncoder().setVelocityConversionFactor(kEncoderPositionFactor / 60);
        rightWheelsMaster.getEncoder().setVelocityConversionFactor(kEncoderPositionFactor / 60);

        leftWheelsMaster.getEncoder().setPositionConversionFactor(kEncoderPositionFactor);
        rightWheelsMaster.getEncoder().setPositionConversionFactor(kEncoderPositionFactor);

        leftWheelsMaster.getPIDController().setP(leftPIDController.getP(), 0);
        leftWheelsMaster.getPIDController().setI(leftPIDController.getI(), 0);
        leftWheelsMaster.getPIDController().setD(leftPIDController.getD(), 0);

        rightWheelsMaster.getPIDController().setP(rightPIDController.getP(), 0);
        rightWheelsMaster.getPIDController().setI(rightPIDController.getI(), 0);
        rightWheelsMaster.getPIDController().setD(rightPIDController.getD(), 0);

//        leftWheelsMaster.burnFlash();
//        leftWheelsSlave.burnFlash();
//        rightWheelsSlave.burnFlash();
//        rightWheelsMaster.burnFlash();
    }

    public void setDutyCycles(double leftDutyCycle, double rightDutyCycle) {
        leftWheelsMaster.set(leftDutyCycle);
        rightWheelsMaster.set(rightDutyCycle);
    }

    public void setVoltages(double leftVoltage, double rightVoltage) {
        leftWheelsMaster.setVoltage(leftVoltage);
        rightWheelsMaster.setVoltage(rightVoltage);
    }

    public void setVelocities(double leftVelocity, double leftFeedForward, double rightVelocity, double rightFeedForward, int sparkMaxPIDSlot) {
        leftWheelsMaster.getPIDController().setReference(leftVelocity, ControlType.kVelocity, sparkMaxPIDSlot, leftFeedForward, CANPIDController.ArbFFUnits.kVoltage);
        rightWheelsMaster.getPIDController().setReference(rightVelocity, ControlType.kVelocity, sparkMaxPIDSlot, rightFeedForward, CANPIDController.ArbFFUnits.kVoltage);
    }

    public void setArcadeSpeeds(double xSpeed, double zRotation) {
        xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
        zRotation = Math.copySign(zRotation * zRotation, zRotation);

        double leftMotorOutput;
        double rightMotorOutput;

        xSpeed = Math
                .max(-1.0 + Math.abs(zRotation),
                        Math.min(1.0 - Math.abs(zRotation), xSpeed));

        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = xSpeed - zRotation;

        setDutyCycles(leftMotorOutput, rightMotorOutput);
    }

    public Rotation2d getHeading() {
        return new Rotation2d(currentHeading);  // counter clock wise positive
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        currentHeading = 0;
    }

    public DifferentialDriveWheelSpeeds getSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                getCANEncoderLeftVelocity(),
                getCANEncoderRightVelocity()
        );
    }

    private void updateRobotPose() {
        robotPose = driveOdometry.update(getHeading(), leftWheelsMaster.getEncoder().getPosition(), rightWheelsMaster.getEncoder().getPosition());
    }

    public void resetHardware() {
        zeroNeoEncoders();
        zeroHeading();
    }

    public void resetOdometry(Pose2d startingPose) {
        driveOdometry.resetPosition(startingPose, getHeading());
    }

    public RamseteController getRamseteController() {
        return ramseteController;
    }

    public DifferentialDriveKinematics getDriveKinematics() {
        return driveKinematics;
    }

    public Pose2d getRobotPose() {
        return robotPose;
    }

    public double getCANEncoderLeftMeters() {
        return leftWheelsMaster.getEncoder().getPosition();
    }

    public double getCANEncoderRightMeters() {
        return rightWheelsMaster.getEncoder().getPosition();
    }

    public double getCANEncoderLeftVelocity() {
        return leftWheelsMaster.getEncoder().getVelocity();
    }

    public double getCANEncoderRightVelocity() {
        return rightWheelsMaster.getEncoder().getVelocity();
    }

    public void zeroNeoEncoders() {
        rightWheelsMaster.getEncoder().setPosition(0);
        leftWheelsMaster.getEncoder().setPosition(0);
    }

    public SimpleMotorFeedforward getDrivetrainFeedforward() {
        return drivetrainFeedforward;
    }

    public PIDController getLeftPIDController() {
        return leftPIDController;
    }

    public PIDController getRightPIDController() {
        return rightPIDController;
    }

}
