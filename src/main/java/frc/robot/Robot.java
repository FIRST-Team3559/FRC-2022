package frc.robot;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.commands.ManualDriveCommand;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;

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
  
  private Thread m_visionThread;
  
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
    
    bottomTunnelMotor.enableDeadbandElimination(true);
    topTunnelMotor.enableDeadbandElimination(true);

    mc_leftRear.follow(mc_leftFront);
    mc_rightRear.follow(mc_rightFront);
    
    DriveSubsystem.getRightEncoder.setPosition(0);
    DriveSubsystem.getLeftEncoder.setPosition(0);
    
    /* Creates a thread which converts color images into grayscale,
    and then detects circle shapes which the robot will go to */
    m_visionThread = new Thread(
      () -> {
        // Starts the camera and sets the resolution, or frame size
        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setResolution(640, 480);
        /* Initializes a sink and allows the Mat to access 
        camera images from the sink */
        CvSink cvSink = CameraServer.getVideo();
        CvSource outputStream = CameraServer.putVideo("Circle", 640, 480);
        Mat mat = new Mat();
        while (!Thread.interrupted()) {
                /* Tell the CvSink to grab a frame from the camera and put it
                in the source mat.  If there is an error notify the output */
                if (cvSink.grabFrame(mat) == 0) {
                  // Send the output the error.
                  outputStream.notifyError(cvSink.getError());
                  // skip the rest of the current iteration
                  continue;
                }
                Imgproc.HoughCircles(mat, new mat.Mat(), mat.HOUGH_GRADIENT, 1, 45, 75, 40, 20, 80);
                // Give the output stream a new image to display
                outputStream.putFrame(mat);
              }
            });
    
    m_visionThread.setDaemon(true);
    m_visionThread.start();

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
    CommandScheduler.getInstance().run();
    
    povForward1.whenHeld(ManualDriveCommand, true);
    povForwardRight1.whenHeld(ManualDriveCommand, true);
    povRight1.whenHeld(ManualDriveCommand, true);
    povBackwardRight1.whenHeld(ManualDriveCommand, true);
    povBackward1.whenHeld(ManualDriveCommand, true);
    povBackwardLeft1.whenHeld(ManualDriveCommand, true);
    povLeft1.whenHeld(ManualDriveCommand, true);
    povForwardLeft1.whenHeld(ManualDriveCommand, true);
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
    ShooterSubsystem.shooterMotor1.disable();
    ShooterSubsystem.shooterMotor2.disable();
    FeederSubsystem.feederMotor.disable();
    TunnelSubsystem.bottomTunnelMotor.disable();
    TunnelSubsystem.topTunnelMotor.disable();
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
