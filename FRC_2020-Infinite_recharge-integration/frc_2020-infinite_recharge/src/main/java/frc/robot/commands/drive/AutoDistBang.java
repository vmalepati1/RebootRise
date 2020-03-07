/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;


public class AutoDistBang extends CommandBase {
  private final Drive drive;
  private final double dist;
  private final Timer cutoff;
  private double cutOffTime = 5.0;
  
  /**
   * Create a new AutoDistBang. Drive distance or until a timer runs out, then stop.
   * Uses non default cutoff time of 5 seconds
   * @param drive
   * @param dist how far to drive in encoder counts
   * @param cutOffTime time to stop even if distance has not been reached.
   */
  public AutoDistBang(Drive drive, double dist, double cutOffTime) {
    this(drive, dist);
    this.cutOffTime = cutOffTime;
  }

  /**
   * Create a new AutoDistBang. Drive distance or until a timer runs out, then stop.
   * Uses default cutoff time of 5 seconds
   * @param drive
   * @param dist how far to drive in encoder counts
   */
  public AutoDistBang(Drive drive, double dist) {
    addRequirements(drive);
    this.drive = drive;
    this.dist = dist;
    cutoff = new Timer();
    cutoff.stop();
    cutoff.reset();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.stop();
    drive.zeroEncoders();
    drive.zeroIMU();
    cutoff.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    drive.zeroEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (drive.getFusedDist() >= dist || cutoff.get() >= cutOffTime) {
      drive.stop();
      drive.zeroEncoders();
      cutoff.stop();
      return true;
    } else {
      drive.straight(0.8);
      return false;
    }
  }
}
