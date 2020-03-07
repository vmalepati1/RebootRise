/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.gamepads;

/**
 * Maps button and axis index numbers to "descriptive" variable names.
 * For use with the WPILIB GenericHID Class.
 * 
 */
public final class F310 {
    // *** Buttons ***
    // Right side (letter) buttons.
    public static final int A_BTN = 1;
    public static final int B_BTN = 2;
    public static final int X_BTN = 3;
    public static final int Y_BTN = 4;

    // Bumpers
    public static final int L_BUMPER = 5;
    public static final int R_BUMPER = 6;

    // Back & Start
    public static final int BACK = 7;
    public static final int START = 8;

    // Stick buttons
    public static final int LS_BTN = 9;
    public static final int RS_BTN = 10;

    // *** Axis ***
    // Sticks
    public static final int LEFT_Y = 1;
    public static final int LEFT_X = 0;
    public static final int RIGHT_Y = 5;
    public static final int RIGHT_X = 4;

    // Triggers
    public static final int LEFT_TRG = 2;
    public static final int RIGHT_TRG = 3;


}
