// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.REVLibError;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * 
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    SmartDashboard.putData("Autonomous", m_chooser);

    {
      if(RobotContainer.leftLeader.setOpenLoopRampRate(.5) !=REVLibError.kOk) {
        SmartDashboard.putString("Ramp Rate", "Error");
      }
    
      if(RobotContainer.leftFollower.setOpenLoopRampRate(.5) !=REVLibError.kOk) {
        SmartDashboard.putString("Ramp Rate", "Error");
      }
    
      if(RobotContainer.rightLeader.setOpenLoopRampRate(.5) !=REVLibError.kOk) {
        SmartDashboard.putString("Ramp Rate", "Error");
      }
    
      if(RobotContainer.rightFollower.setOpenLoopRampRate(.5) !=REVLibError.kOk) {
        SmartDashboard.putString("Ramp Rate", "Error");
      }
    }

    CameraServer.startAutomaticCapture();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    RobotContainer.m_leftEncoder.setPosition(Constants.position);
    RobotContainer.m_rightEncoder.setPosition(Constants.position);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kDefaultAuto:
      default:
      RobotContainer.shooter();
      RobotContainer.driveTrain.tankDrive(-.7, -.7);
      Constants.position = RobotContainer.m_leftEncoder.getPosition();
      if(Constants.position == 5) {
        RobotContainer.leftLeader.stopMotor();
        RobotContainer.rightLeader.stopMotor();
        RobotContainer.feeder();
        RobotContainer.shooter();
        Constants.position = 0;
        RobotContainer.m_leftEncoder.setPosition(Constants.position);
        RobotContainer.driveTrain.tankDrive(.7, .7);
        while (Constants.position != 5) {
          Constants.position = RobotContainer.m_leftEncoder.getPosition();
        }
        if(Constants.position == 5) {
          RobotContainer.leftLeader.stopMotor();
          RobotContainer.rightLeader.stopMotor();
          RobotContainer.shooter();
        }
      }
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    RobotContainer.driveTrain.tankDrive(RobotContainer.leftStick.getRawAxis(1), RobotContainer.rightStick.getRawAxis(5));
    RobotContainer.feeder();
    RobotContainer.tunnel();
    RobotContainer.shooter();
    }
  

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}