/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
// import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Shooter extends SubsystemBase {

  private final CANSparkMax shootMaster, shootFollow;

  private final TalonSRX angle;
  private final SensorCollection angleSensor;
  // private final DigitalInput lowAngle, highAngle; plug into tallon

  /**
   * Rev Sprak Max/Shooter variables
   */
  // private final CANPIDController pidController;
  private final CANEncoder encoder;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private final double maxRPM = 5750;
  private final int maxAngle = 819; // ~ 36 deg
  private final int angleDB = 6; // ~ 0.3 deg
  private final double angleSpeed = 0.2;
  
  private double speedSP;
  private int angleSP;
  
  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    // setpoints
    speedSP = 0;
    angleSP = 0;
    
    // shooter motors
    shootMaster = new CANSparkMax(Constants.SHOOTER_MASTER_CAN, MotorType.kBrushless);
    shootFollow = new CANSparkMax(Constants.SHOOTER_FOLLOWER_CAN, MotorType.kBrushless);
    // pidController = shootMaster.getPIDController();
    encoder = shootMaster.getEncoder();
    configMAX();

    // hood angle motor
    angle = new TalonSRX(Constants.SHOOTER_ANGLE_CAN);
    configAngleSRX();
    angleSensor = angle.getSensorCollection();

    // shooter pid
    kP = 5e-2;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.0001;
    kMaxOutput = 1; 
    kMinOutput = -1;
  }

  private void configMAX() {
    shootMaster.setIdleMode(IdleMode.kCoast);
    shootFollow.setIdleMode(IdleMode.kCoast);
    shootFollow.follow(shootMaster, true);
    shootMaster.set(0);
    /* pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput); */
  }

  private void configAngleSRX() {
    angle.setNeutralMode(NeutralMode.Brake);
    angle.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    angle.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    angle.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
    /* Ensure sensor is positive when output is positive */
    angle.setSensorPhase(true);
    angleSensor.setQuadraturePosition(0, 0);
    angleSensor.getQuadraturePosition();
    /* angle.configNominalOutputForward(0, 0);
		angle.configNominalOutputReverse(0, 0);
		angle.configPeakOutputForward(0.7, 0);
    angle.configPeakOutputReverse(-0.7, 0);
    angle.config_kP(0, 1);
    angle.config_kI(0, 0);
    angle.config_kD(0, 0); */
  }

  /**
   * This method will be called once per scheduler run
   * bang bang shooter speed control
   * bang bang shooter hood angle contorl
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    bbShoot();
    bbAngle();
  }

  private void bbShoot() {
    if (speedSP > 0) {
      if (speedSP < encoder.getVelocity()) {
        shootMaster.set(1);
      } else {
        shootMaster.set(0);
      }
    } else  {
      shootMaster.set(0);
    }
  }

  /**
   * Set the angle with bang bang methond
   */
  private void bbAngle() {
    if (angleSensor.isFwdLimitSwitchClosed()) {
      angleSensor.setQuadraturePosition(819, 0);
    } else if (angleSensor.isRevLimitSwitchClosed()) {
      angleSensor.setQuadraturePosition(0, 0);
    }

    if (inDeadBand(angleSensor.getQuadraturePosition(), angleSP, angleDB)) {
      angle.set(ControlMode.PercentOutput, 0);
    } else {
      if (angleSP > angleSensor.getQuadraturePosition()) {
        angle.set(ControlMode.PercentOutput, angleSpeed);
      } else if (angleSP < angleSensor.getQuadraturePosition()) {
        angle.set(ControlMode.PercentOutput, -angleSpeed);
      }
    }
  }

  /**
   * @param angle angle to target
   * @return Is hood at angle?
   */
  public boolean setAngle(int angle) {
    if (angle > maxAngle) {
      angle = maxAngle;
    } else if (angle < 0) {
      angle = 0;
    }
    angleSP = angle;
    if (inDeadBand(angleSensor.getQuadraturePosition(), angle, angleDB)) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * 
   * @param speed what speed do you want to set?
   * @return boolean: Is the moter at speed?
   */
  public boolean setSpeed(double speed) {
    if(speed > maxRPM) {
      speed = maxRPM;
    } else if (speed < 0) {
      speed = 0;
    }
    speedSP = speed;
    if (encoder.getVelocity() >= speed) {
      return true;
    } else {
      return false;
    }
  }

  public void stop() {
    shootMaster.set(0);
    speedSP = 0;
  }

  /**
   * 
   * @return boolean: Is the moter at speed?
   */
  public boolean maxSpeed() {
    return setSpeed(maxRPM);
  }

  /**
   * 
   * @param kX current value
   * @param target target value
   * @param deadBand
   * @return is value
   */
  private static boolean inDeadBand(int kX, int target, int deadBand) {
    int bandMax = target + (deadBand/2); 
    int bandMin = target - (deadBand/2);
    if (kX >= bandMin && kX <= bandMax) {
      return true;
    }
    return false;
  }

  public double getSpeedSP() {
    return this.speedSP;
  }
}
