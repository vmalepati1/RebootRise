package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.utils.EnhancedJoystickButton;
import frc.robot.utils.Gamepad;

import static frc.robot.Constants.Joysticks.*;

public class OI {

    public static Joystick leftJoystick = new Joystick(LEFT_JOYSTICK_PORT);
    public static Joystick rightJoystick = new Joystick(RIGHT_JOYSTICK_PORT);
    public static Gamepad gamepad = new Gamepad(GAMEPAD_PORT);

    public static EnhancedJoystickButton shootButton = new EnhancedJoystickButton(
            new EnhancedJoystickButton.EnhancedButtonIndex(gamepad, 2));
    public static EnhancedJoystickButton barfButton = new EnhancedJoystickButton(
            new EnhancedJoystickButton.EnhancedButtonIndex(gamepad, 3));
    public static EnhancedJoystickButton shootToggleUpButton = new EnhancedJoystickButton(
            new EnhancedJoystickButton.EnhancedButtonIndex(gamepad, 4));
    public static EnhancedJoystickButton shootToggleDownButton = new EnhancedJoystickButton(
            new EnhancedJoystickButton.EnhancedButtonIndex(gamepad, 1));
    public static EnhancedJoystickButton spinToShootButton = new EnhancedJoystickButton(
            new EnhancedJoystickButton.EnhancedButtonIndex(gamepad, 5));
    public static EnhancedJoystickButton spinToIntakeButton = new EnhancedJoystickButton(
            new EnhancedJoystickButton.EnhancedButtonIndex(gamepad, 6));

}
