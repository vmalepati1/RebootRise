/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * 
     */
    // 40 amp
    public static final int LEFT_DRIVE1_CAN=10;
    public static final int LEFT_DRIVE2_CAN=11;
    public static final int RIGHT_DRIVE1_CAN=12;
    public static final int RIGHT_DRIVE2_CAN=13;

    public static final int LIFT_MOTOR_CAN=14;
    public static final int BALANCE_MOTOR_CAN=15;
    
    // 30 amp
    public static final int SHOOTER_MASTER_CAN=20;
    public static final int SHOOTER_FOLLOWER_CAN=21;
    public static final int SHOOTER_ANGLE_CAN=22;

    public static final int INDEXER_CAN=30;

    // Limit Switches
    public static final int BALL_SENSOR=0;
    public static final int INTAKE_SENSOR=1; // hopper position
    public static final int SHOOT_SENSOR=2;
    public static final int LIFT_TOP_SWITCH=3;
    public static final int LIFT_BOTTOM_SWITCH=4;
    public static final int LOADER_SENSOR=5;

    // Solenoids
    public static final int RAMP_RELEASE=0;
    public static final int LOADER=1;

    // Sensors
    public static final int IMU_CAN=40;
        
}

