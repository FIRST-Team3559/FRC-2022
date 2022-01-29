package frc.robot;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  private DifferentialDrive driveBase;
  private Joystick driverGamepad;
  
  public Robot() {
    DriveSubsystem.registerSubsystem(DriveSubsystem);
    FeederSubsystem.registerSubsystem(FeederSubsystem);
    TunnelSubsystem.registerSubsystem(TunnelSubsystem);
    ShooterSubsystem.registerSubsystem(ShooterSubsystem);
  }

  /**
   * 
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    mc_leftRear.follow(mc_leftFront);
    mc_rightRear.follow(mc_rightFront);
    
    DriveSubsystem.getRightEncoder.setPosition(0);
    DriveSubsystem.getLeftEncoder.setPosition(0);

    driveBase = new DifferentialDrive(leftLeader, rightLeader);

    driverGamepad = new Joystick(Constants.gamePadPort);

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

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  DriveSubsystem.periodic();
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
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    driveBase.tankDrive(-driverGamepad.getRawAxis(1), driverGamepad.getRawAxis(5));
    FeederSubsystem.feeder();

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    DriveSubsystem.mcg_left.disable();
    DriveSubsystem.mcg_right.disable();
  }

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
}
