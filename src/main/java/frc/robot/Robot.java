// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.REVLibError;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

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

  public static CANSparkMax leftLeader = new CANSparkMax(Constants.leftLeaderDeviceID, MotorType.kBrushless);
  public static CANSparkMax leftFollower = new CANSparkMax(Constants.leftFollowerDeviceID, MotorType.kBrushless);
  public static MotorControllerGroup mcg_left = new MotorControllerGroup(leftLeader, leftFollower);

  public static CANSparkMax rightLeader = new CANSparkMax(Constants.rightLeaderDeviceID, MotorType.kBrushless);
  public static CANSparkMax rightFollower = new CANSparkMax(Constants.rightFollowerDeviceID, MotorType.kBrushless);
  public static MotorControllerGroup mcg_right = new MotorControllerGroup(rightLeader, rightFollower);
  public static DifferentialDrive driveTrain = new DifferentialDrive(mcg_left, mcg_right);

  public final static RelativeEncoder m_leftEncoder = leftLeader.getEncoder(Constants.kHallSensor, Constants.countsPerRev);
  public final static RelativeEncoder m_rightEncoder = rightLeader.getEncoder(Constants.kHallSensor, Constants.countsPerRev);
  private static double position = 0;

  public static Spark feederMotor = new Spark (Constants.feederChannel);
  public static Spark shooterMotor1 = new Spark(Constants.shooterMotor1Channel);
  public static Spark shooterMotor2 = new Spark(Constants.shooterMotor2Channel);
  public static Spark ballTunnelMotor = new Spark(Constants.ballTunnelChannel);

  public static Joystick leftStick = new Joystick(0);
  public static Joystick rightStick = new Joystick(1);
  public static Joystick operatorStick = new Joystick(2);

  /**
   * 
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    SmartDashboard.putData("Autonomous", m_chooser);
    SmartDashboard.putNumber("Drive Train RPM", m_leftEncoder.getVelocity());
    SmartDashboard.putNumber("Position", position);
    SmartDashboard.putNumber("Shooter Motor 1 Speed", shooterMotor1.get());
    SmartDashboard.putNumber("Shooter Motor 2 Speed", shooterMotor2.get());
    SmartDashboard.putNumber("Tunnel Motor Speed", ballTunnelMotor.get());
    SmartDashboard.putNumber("Feeder Motor Speed", feederMotor.get());

    {
      if(leftLeader.setOpenLoopRampRate(.5) !=REVLibError.kOk) {
        SmartDashboard.putString("Ramp Rate", "Error");
      }
    
      if(leftFollower.setOpenLoopRampRate(.5) !=REVLibError.kOk) {
        SmartDashboard.putString("Ramp Rate", "Error");
      }
    
      if(rightLeader.setOpenLoopRampRate(.5) !=REVLibError.kOk) {
        SmartDashboard.putString("Ramp Rate", "Error");
      }
    
      if(rightFollower.setOpenLoopRampRate(.5) !=REVLibError.kOk) {
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
    m_leftEncoder.setPosition(position);
    m_rightEncoder.setPosition(position);
    shooter();
    driveTrain.tankDrive(-.7, -.7);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kDefaultAuto:
      default:
      position = m_leftEncoder.getPosition();
      if(position == 5) {
        leftLeader.stopMotor();
        rightLeader.stopMotor();
        feeder();
        shooter();
        position = 0;
        m_leftEncoder.setPosition(position);
        driveTrain.tankDrive(.7, .7);
        while (position != 5) {
          position = m_leftEncoder.getPosition();
        }
        if(position == 5) {
          leftLeader.stopMotor();
          rightLeader.stopMotor();
          shooter();
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
    position = m_leftEncoder.getPosition();
    driveTrain.tankDrive(leftStick.getRawAxis(1), rightStick.getRawAxis(5));
    feeder();
    tunnel();
    shooter();
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

  public static void tunnel() {
    if (operatorStick.getRawButton(3)) {
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
    if (operatorStick.getRawButton(2)) {
        shooterMotor1.set(.9);
    }
  }

}