package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class DriveBaseSubsystem extends SubsystemBase {
  // Motor controllers Left
  public static CANSparkMax leftLeader = new CANSparkMax(Constants.leftLeaderDeviceID, MotorType.kBrushless);
  public static CANSparkMax leftFollower = new CANSparkMax(Constants.leftFollowerDeviceID, MotorType.kBrushless);
  public MotorControllerGroup mcg_left = new MotorControllerGroup(leftLeader, leftFollower);
  // Motor controllers Right
  public static CANSparkMax rightLeader = new CANSparkMax(Constants.rightLeaderDeviceID, MotorType.kBrushless);
  public static CANSparkMax rightFollower = new CANSparkMax(Constants.rightFollowerDeviceID, MotorType.kBrushless);
  public MotorControllerGroup mcg_right = new MotorControllerGroup(rightLeader, rightFollower);
  
  public DifferentialDrive drivetrain = new DifferentialDrive(mcg_left, mcg_right);
  private final static RelativeEncoder m_leftEncoder = leftLeader.getEncoder(Constants.kHallSensor, Constants.countsPerRev);
  private final static RelativeEncoder m_rightEncoder = rightLeader.getEncoder(Constants.kHallSensor, Constants.countsPerRev);
  public static ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(Constants.kTrackwidthMeters);
  public static Rotation2d gyroAngle = new Rotation2d(0);
  public static DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(gyroAngle);
  public static Pose2d m_pose;
  public static DifferentialDrive driveBase;
  
  /** Creates a new DriveSubsystem. 
   * @return */
  public void DriveSubsystem() {
    // Sets the distance per pulse for the encoders
    ((Encoder) m_leftEncoder).setDistancePerPulse(Constants.kEncoderDistancePerPulse);
    ((Encoder) m_rightEncoder).setDistancePerPulse(Constants.kEncoderDistancePerPulse);
    
    resetEncoders();
  }
   
  @Override
public void periodic() {
  // Get my gyro angle. We are negating the value because gyros return positive
  // values as the robot turns clockwise. This is not standard convention that is
  // used by the WPILib classes.
  gyroAngle = Rotation2d.fromDegrees(gyro.getAngle());

  // Update the pose
  m_pose = m_odometry.update(gyroAngle, m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
}
  
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  // Resets the drive encoders to currently read a position of 0.
  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  // Gets the average distance of the two encoders.
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
  }

  public RelativeEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  public RelativeEncoder getRightEncoder() {
    return m_rightEncoder;
  }
}
