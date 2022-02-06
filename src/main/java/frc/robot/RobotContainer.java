// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
   private final XboxController m_joystick1 = new XboxController(0);
   public final static POVButton povForward1 = new POVButton(m_joystick1, 0, 0);
   public final static POVButton povForwardRight1 = new POVButton(m_joystick1, 45, 1);
   public final static POVButton povRight1 = new POVButton(m_joystick1, 90, 2);
   public final static POVButton povBackwardRight1 = new POVButton(m_joystick1, 135, 3);
   public final static POVButton povBackward1 = new POVButton(m_joystick1, 180, 4);
   public final static POVButton povBackwardLeft1 = new POVButton(m_joystick1, 225, 5);
   public final static POVButton povLeft1 = new POVButton(m_joystick1, 270, 6);
   public final static POVButton povForwardLeft1 = new POVButton(m_joystick1, 315, 7);
   povForward1.whenPressed(ManualDriveCommand, true);
   povForwardRight1.whenPressed(ManualDriveCommand, true);
   povRight1.whenPressed(ManualDriveCommand, true);
   povBackwardRight1.whenPressed(ManualDriveCommand, true);
   povBackward1.whenPressed(ManualDriveCommand, true);
   povBackwardLeft1.whenPressed(ManualDriveCommand, true);
   povLeft1.whenPressed(ManualDriveCommand, true);
   povForwardLeft1.whenPressed(ManualDriveCommand, true);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoCommand;
  }
}
