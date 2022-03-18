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
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "Shoot Only";
  private static final String kNoAuto = "Do Nothing";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public static CANSparkMax leftLeader = new CANSparkMax(Constants.leftLeaderDeviceID, MotorType.kBrushless);
  public static CANSparkMax leftFollower = new CANSparkMax(Constants.leftFollowerDeviceID, MotorType.kBrushless);
  public static MotorControllerGroup mcg_left = new MotorControllerGroup(leftLeader, leftFollower);

  public static CANSparkMax rightLeader = new CANSparkMax(Constants.rightLeaderDeviceID, MotorType.kBrushless);
  public static CANSparkMax rightFollower = new CANSparkMax(Constants.rightFollowerDeviceID, MotorType.kBrushless);
  public static MotorControllerGroup mcg_right = new MotorControllerGroup(rightLeader, rightFollower);
  public static DifferentialDrive driveTrain = new DifferentialDrive(mcg_left, mcg_right);

  public static CANSparkMax winchMotor = new CANSparkMax(Constants.winchMotorID, MotorType.kBrushless);
  public static Spark feederMotor = new Spark (Constants.feederChannel);
  public static Spark highShooterMotor = new Spark(Constants.highShooterMotorChannel);
  public static Spark lowShooterMotor = new Spark(Constants.lowShooterMotorChannel);
  public static Spark ballTunnelMotor = new Spark(Constants.ballTunnelChannel);

  public static Joystick leftStick = new Joystick(0);
  public static Joystick rightStick = new Joystick(1);
  public static Joystick operatorStick = new Joystick(2);

  private Timer timer;

  private static double shooterSpeed;
  private static double climberSpeed;

  /**
   * 
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Shoot Only", kCustomAuto);
    m_chooser.addOption("Do Nothing", kNoAuto);
    SmartDashboard.putData("Autonomous", m_chooser);

    leftLeader.setOpenLoopRampRate(.5);
    leftFollower.setOpenLoopRampRate(.5);
    rightLeader.setOpenLoopRampRate(.5);
    rightFollower.setOpenLoopRampRate(.5);

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

    timer = new Timer();

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
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    shooterSpeed = .9;
    switch (m_autoSelected) {
      case kCustomAuto:
       if (timer.get() < 3) {
         highShooterMotor.set(shooterSpeed);
         lowShooterMotor.set(-shooterSpeed);
         ballTunnelMotor.set(-.25); 
       } else {
        highShooterMotor.set(0);
        lowShooterMotor.set(0);
        ballTunnelMotor.set(0); 
       }
       break;
      case kNoAuto:
       break;
      case kDefaultAuto:
      default:
       if (timer.get() < 3) {
        highShooterMotor.set(shooterSpeed);
        lowShooterMotor.set(-shooterSpeed);
        ballTunnelMotor.set(-.25);
       } else if (timer.get() < 6) {
        ballTunnelMotor.set(0);
        highShooterMotor.set(0);
        lowShooterMotor.set(0);
        feederMotor.set(0.5);
        driveTrain.tankDrive(0.5, -0.5);
       } else if (timer.get() < 9) {
        driveTrain.tankDrive(-0.5, 0.5);
        feederMotor.set(0);
       } else if (timer.get() < 12) {
        driveTrain.tankDrive(0, 0);
        highShooterMotor.set(shooterSpeed);
        lowShooterMotor.set(-shooterSpeed);
        ballTunnelMotor.set(-.25);
       } else if (timer.get() < 15) {
         driveTrain.tankDrive(0.6, -0.6);
         ballTunnelMotor.set(0);
         highShooterMotor.set(0);
         lowShooterMotor.set(0);
       } else {
        driveTrain.tankDrive(0, 0);
        ballTunnelMotor.set(0);
        highShooterMotor.set(0);
        lowShooterMotor.set(0);
       }
        break;
      }
    }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    driveTrain.tankDrive(leftStick.getRawAxis(1), rightStick.getRawAxis(1));
    feeder();
    tunnel();
    shooter();
    climber();
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
    if (operatorStick.getTrigger()) {
      if (operatorStick.getRawAxis(1) > .75) {
        shooterSpeed = .75 + operatorStick.getRawAxis(1);
        } else if (operatorStick.getRawAxis(1) < -.75) {
          shooterSpeed = .75 + operatorStick.getRawAxis(1);
        } else {
          shooterSpeed = .9;
        }

        highShooterMotor.set(shooterSpeed);
        lowShooterMotor.set(-shooterSpeed);
    } else {
      highShooterMotor.set(0);
      lowShooterMotor.set(0);
    }
  }

  public static void climber() {
    if (!operatorStick.getTrigger()) {
      if (operatorStick.getRawAxis(1) > .5) {
        climberSpeed = operatorStick.getRawAxis(1);
          winchMotor.set(-climberSpeed);
      } else if (operatorStick.getRawAxis(1) < -.5) {
        climberSpeed = operatorStick.getRawAxis(1);
        winchMotor.set(climberSpeed);
      } else {
        winchMotor.set(0);
      }
    }
  }
}