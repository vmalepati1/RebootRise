/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.InitAuto;
import frc.robot.commands.ShootOne;
import frc.robot.commands.indexer.Advance;
import frc.robot.commands.indexer.Load;
import frc.robot.commands.indexer.AutoIndex;
import frc.robot.commands.shooter.ToggleShooter;
import frc.robot.gamepads.F310;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // ** Joysticks **
  private static final GenericHID opJoystick = new Joystick(0);
  private static final GenericHID driverJoy = new Joystick(1);

  // ** buttons **
  private final JoystickButton opBBtn = new JoystickButton(opJoystick, F310.B_BTN);
  private final JoystickButton opABtn = new JoystickButton(opJoystick, F310.A_BTN);
  private final JoystickButton opYBtn = new JoystickButton(opJoystick, F310.Y_BTN);
  private final JoystickButton opXBtn = new JoystickButton(opJoystick, F310.X_BTN);
  private final JoystickButton opLBump = new JoystickButton(opJoystick, F310.L_BUMPER);
  private final JoystickButton opRBump = new JoystickButton(opJoystick, F310.R_BUMPER);

  // private final JoystickButton drBBtn = new JoystickButton(driverJoy, F310.B_BTN);
  // private final JoystickButton drABtn = new JoystickButton(driverJoy, F310.A_BTN);
  // private final JoystickButton drYBtn = new JoystickButton(driverJoy, F310.Y_BTN);
  // private final JoystickButton drXBtn = new JoystickButton(driverJoy, F310.X_BTN);
  // private final JoystickButton drLBump = new JoystickButton(driverJoy, F310.L_BUMPER);
  // private final JoystickButton drRBump = new JoystickButton(driverJoy, F310.R_BUMPER);

  // ** subsystems **
  private final Index index = new Index();
  private final Drive drive = new Drive(driverJoy);
  // private final Shooter shooter = new Shooter();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // opABtn.whenPressed(new Load(index));
    // opBBtn.whileHeld(new AutoIndex(index));
    // opYBtn.whenPressed(new ToggleShooter(shooter));
    // opXBtn.whenPressed(new ShootOne(shooter, index));
    // opLBump.whenPressed(new Advance(index, Advance.Position.HOPPER));
    // opRBump.whenPressed(new Advance(index, Advance.Position.SHOOT));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InitAuto(index);
  }

  public Drive getDrive() {
    return drive;
  }
}
