/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.auton.*;
import frc.robot.commands.teleop.DriveCommand;
import frc.robot.commands.teleop.IndexerCommand;
import frc.robot.commands.teleop.ShooterCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.LiveDashboardHelper;

import static frc.robot.Constants.SmartDashboardKeys.AUTON_SELECT_ID;
import static frc.robot.Constants.SmartDashboardKeys.IS_BLUE;
import static frc.robot.OI.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static Drivetrain drivetrain;
  public static Shooter shooter;
  public static Indexer indexer;

  public static boolean isBlue = true;
  public static boolean isAuto = true;

//  private static SendableChooser<SequentialCommandGroup> autonChooser;

//  private Solenoid testSolenoid = new Solenoid(0);

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    drivetrain = new Drivetrain();
    shooter = new Shooter();
    indexer = new Indexer();

    SmartDashboard.putBoolean(IS_BLUE, false);
    SmartDashboard.putNumber(AUTON_SELECT_ID, 0);

    CommandScheduler.getInstance().setDefaultCommand(drivetrain, new DriveCommand());
    CommandScheduler.getInstance().setDefaultCommand(shooter, new ShooterCommand());
    CommandScheduler.getInstance().setDefaultCommand(indexer, new IndexerCommand());

//    autonChooser = new SendableChooser<>();
//    autonChooser.addOption("Do Nothing", new SequentialCommandGroup());
//    autonChooser.addOption("Cross Baseline Forwards", new TimedAuton(
//            new DriveStraightTime(0.5, 2.5)));
//    autonChooser.setDefaultOption("Cross Baseline Backwards", new TimedAuton(
//            new AlignToShoot(),
//            new SpinToShoot(true),
//            new ShootOneBall(2),
//            new InstantCommand(() -> indexer.setShootToggle(false)),
//            new WaitCommand(0.25),
//            new SpinToShoot(true),
//            new ShootOneBall(2),
//            new InstantCommand(() -> indexer.setShootToggle(false)),
//            new WaitCommand(0.25),
////            new ShiftOneToShoot(),
////            new ShootOneBall(5),
////            new ShiftOneToShoot(),
////            new ShootOneBall(5),
//            new DriveStraightTime(-0.5, 1.5),
//            new InstantCommand(() -> drivetrain.setDutyCycles(0, 0))));
//    autonChooser.addOption("Ramsete Straight Test", new TimedAuton(
//            new ResetPose(Paths.TestTrajectories.testForward.getInitialPose()),
//            new RamseteTrackingCommand(Paths.TestTrajectories.testForward),
//            new InstantCommand(() -> drivetrain.setDutyCycles(0, 0))));
//    SmartDashboard.putData("Auton Selector", autonChooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (DriverStation.getInstance().getAlliance() != DriverStation.Alliance.Invalid) {
      isBlue = SmartDashboard.getBoolean(IS_BLUE, false);
    } else {
      isBlue = (DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue);
    }
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit () {
//    SmartDashboard.putData("Auton Selector", autonChooser);
  }

  @Override
  public void disabledPeriodic () {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    isAuto = true;
    drivetrain.resetHardware();
//    autonChooser.getSelected().schedule();
    getAutonCommand().schedule();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic () {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit () {
    isAuto = false;
    drivetrain.resetHardware();
    shooter.setupFlywheel();
    shootToggleDownButton.whenPressed(() -> indexer.setShootToggle(false));
    shootToggleUpButton.whenPressed(() -> indexer.setShootToggle(true));
    spinToShootButton.whenPressed(new SpinToShoot(true));
    spinToIntakeButton.whenPressed(new SpinToIntake(true));
//    testSolenoid.set(true);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic () {
    CommandScheduler.getInstance().run();

    LiveDashboardHelper.putRobotData(drivetrain.getRobotPose());
  }

  @Override
  public void testInit () {
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic () {
  }

  private SequentialCommandGroup getAutonCommand() {
    return new TimedAuton(
            new InstantCommand(() -> shooter.autoShouldShoot = true),
            new AlignToShoot(),
            new SpinToShoot(true),
            new InstantCommand(() -> indexer.setShootToggle(true)),
            new WaitCommand(0.25),
            new InstantCommand(() -> indexer.setShootToggle(false)),
            new WaitCommand(0.25),
            new SpinToShoot(true),
            new ShootWhenReady().withTimeout(2),
            new WaitCommand(0.25),
            new InstantCommand(() -> indexer.setShootToggle(false)),
            new WaitCommand(0.25),
            new SpinToShoot(true),
            new ShootWhenReady().withTimeout(2),
            new WaitCommand(0.25),
            new InstantCommand(() -> indexer.setShootToggle(false)),
            new WaitCommand(0.25),
            new InstantCommand(() -> shooter.autoShouldShoot = false),
            new DriveStraightTime(-0.5, 1.5),
            new InstantCommand(() -> drivetrain.setDutyCycles(0, 0)));
  }
}
