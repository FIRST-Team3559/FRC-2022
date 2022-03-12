// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static CANSparkMax leftLeader = new CANSparkMax(Constants.leftLeaderDeviceID, MotorType.kBrushless);
  public static CANSparkMax leftFollower = new CANSparkMax(Constants.leftFollowerDeviceID, MotorType.kBrushless);
  public static MotorControllerGroup mcg_left = new MotorControllerGroup(leftLeader, leftFollower);
  public static CANSparkMax rightLeader = new CANSparkMax(Constants.rightLeaderDeviceID, MotorType.kBrushless);
  public static CANSparkMax rightFollower = new CANSparkMax(Constants.rightFollowerDeviceID, MotorType.kBrushless);
  public static MotorControllerGroup mcg_right = new MotorControllerGroup(rightLeader, rightFollower);
  public static DifferentialDrive driveTrain = new DifferentialDrive(mcg_left, mcg_right);
  public final static RelativeEncoder m_leftEncoder = leftLeader.getEncoder(Constants.kHallSensor, Constants.countsPerRev);
  public final static RelativeEncoder m_rightEncoder = rightLeader.getEncoder(Constants.kHallSensor, Constants.countsPerRev);
  public static Spark feederMotor = new Spark (Constants.feederChannel);
  public static Spark shooterMotor1 = new Spark(Constants.shooterMotor1Channel);
  public static Spark shooterMotor2 = new Spark(Constants.shooterMotor2Channel);
  public static Spark ballTunnelMotor = new Spark(Constants.ballTunnelChannel);

  public static Joystick leftStick = new Joystick(0);
  public static Joystick rightStick = new Joystick(1);
  public static Joystick operatorStick = new Joystick(2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  public static void tunnel() {
    if (RobotContainer.operatorStick.getRawButton(3)) {
      ballTunnelMotor.set(-.7);
    } else {
      ballTunnelMotor.set(0);
    }
  }

  public static void feeder() {
    if (operatorStick.getRawButton(5)) {
      feederMotor.set(0.5);
    } else {
    if (operatorStick.getRawButton(4)) {
      feederMotor.set(-.5);
    } else {
      feederMotor.set(0);
    }
  }
}

  public static void shooter() {
    double[] motorSpeeds = {.1, .15, .2, .25, .3, .35, .4, .45, .5, .55, .6, .65, .7, .8, .9};
    
    if (RobotContainer.operatorStick.getRawButton(2)) {
      for (int i = 0; i < motorSpeeds.length; i++) {
        shooterMotor1.set(i);
      }
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
