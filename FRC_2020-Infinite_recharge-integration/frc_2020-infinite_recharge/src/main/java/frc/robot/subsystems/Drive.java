/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.gamepads.F310;

public class Drive extends SubsystemBase {

  private final CANSparkMax leftMaster, leftFollow, rightMaster, rightFollow;
  private final int stallLimit = 40, freeLimit = 20;
  private final GenericHID driverJoy;
  // PigeonIMU imu = new PigeonIMU(Constants.IMU_CAN);
  private final CANSparkMax[] allMCs = new CANSparkMax[4];

  /**
   * Creates a new Drive.
   */
  public Drive(final GenericHID driverJoy) {
    this.driverJoy = driverJoy;

    leftMaster = new CANSparkMax(Constants.LEFT_DRIVE1_CAN, MotorType.kBrushless);
    leftFollow = new CANSparkMax(Constants.LEFT_DRIVE2_CAN, MotorType.kBrushless);
    rightMaster = new CANSparkMax(Constants.RIGHT_DRIVE1_CAN, MotorType.kBrushless);
    rightFollow = new CANSparkMax(Constants.RIGHT_DRIVE2_CAN, MotorType.kBrushless);

    allMCs[0] = leftMaster;
    allMCs[1] = leftFollow;
    allMCs[2] = rightFollow;
    allMCs[3] = rightMaster;
    
    for (CANSparkMax csm : allMCs) {
      csm.set(0);
      csm.setIdleMode(IdleMode.kBrake);
    }

    leftMaster.setInverted(true);
//    leftMaster.getEncoder().setInverted(true);
//    leftFollow.getEncoder().setInverted(true);

    leftFollow.follow(leftMaster);
    rightFollow.follow(rightMaster);


    leftMaster.setSmartCurrentLimit(stallLimit, freeLimit);
    rightMaster.setSmartCurrentLimit(stallLimit, freeLimit);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void diffDrive() {
    leftMaster.set(driverJoy.getRawAxis(F310.LEFT_Y));
    rightMaster.set(driverJoy.getRawAxis(F310.RIGHT_Y));
  }

  /**
   * Drive in a straight line
   * @param speed % power -1 to 1
   */
  public void straight(double speed) {
    leftMaster.set(speed);
    rightMaster.set(speed);
  }

  private void fieldCentric(double jLeft, double jRight) {

  }

  public GenericHID getJoy() {
    return driverJoy;
  }

  public void zeroIMU() {
  }

  // public double getHeading() {
  //   return imu.getFusedHeading();
  // }

  public void zeroEncoders() {
    for (CANSparkMax csm : allMCs) {
      csm.getEncoder().setPosition(0);
    }
  }

  /**
   * Average encoder value for all four motors
   * @return double: averaged encoder values
   */
  public double getFusedDist() {
    double avg = 0;
    for (CANSparkMax csm : allMCs) {
      avg += csm.getEncoder().getPosition(); 
    }
    return avg/allMCs.length;
  }

  public void resetEncoders() {
    for (CANSparkMax csm : allMCs) {
      csm.getEncoder().setPosition(0);
    }
  }

  public void stop() {
    leftMaster.set(0);
    rightMaster.set(0);
  }
}
