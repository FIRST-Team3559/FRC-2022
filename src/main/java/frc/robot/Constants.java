// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // SparkMAX CAN IDs for driveBase
    public static int FL_MOTOR_CONTROLLER_ID = 10;
    public static int RL_MOTOR_CONTROLLER_ID = 11;
    public static int FR_MOTOR_CONTROLLER_ID = 12;
    public static int RR_MOTOR_CONTROLLER_ID = 13;
    
    public static int feederChannel = 0;
    
    public static int gamePadPort = 0;
    
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    public static final double ksVolts;
    public static final double kvVoltSecondsPerMeter;
    public static final double kaVoltSecondsSquaredPerMeter;
    public static final double kPDriveVel;
    public static final double kTrackwidthMeters;
    public static final double kMaxSpeedMetersPerSecond;
    public static final double kMaxAccelerationMetersPerSecondSquared;
}
