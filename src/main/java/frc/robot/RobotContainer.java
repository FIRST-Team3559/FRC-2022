// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.examples.ramsetecommand.Constants.DriveConstants;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer extends Subsystembase {
  // The robot's subsystems and commands are defined here...
  private final XboxController m_joystick1 = new XboxController(0);
  public final static Joystick stick0 = new Joystick(0);
  public final static Joystick stick1 = new Joystick(1);
  public final static POVButton povForward = new POVButton(stick0, 0);
  public final static POVButton povRight = new POVButton(stick0, 90);
  public final static POVButton povBackward = new POVButton(stick0, 180);
  public final static POVButton povLeft = new POVButton(stick0, 270);
  private final Encoder m_leftEncoder = new Encoder (DriveConstants.kLeftEncoderPorts[0], DriveConstants.kLeftEncoderPorts[1], DriveConstants.kLeftEncoderReversed);
  private final Encoder m_rightEncoder = new Encoder (DriveConstants.kRightEncoderPorts[0], DriveConstants.kRightEncoderPorts[1], DriveConstants.kRightEncoderReversed);
  private final Gyro m_gyro = new ADXRS450_Gyro();
  private final DifferentialDriveOdometry m_odometry;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    // Sets the length of the pulse of each encoder, a pulse being an encoder cycle
    m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }
  
  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoCommand;
  }
  
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }
  
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
  
  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }
  
  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }
  
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }
  
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }
  
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  
  public void zeroHeading() {
    // Sets the direction the robot is facing as 0; makes it the new starting point
    m_gyro.reset();
  }
  
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }
 
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }
}
