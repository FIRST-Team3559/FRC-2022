package java.frc.robot;

import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DifferentialDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class DriveBaseSubsystem extends SubsystemBase {
   // Motor controllers Left
  public CANSparkMax mc_leftFront = new CANSparkMax(Constants.FL_MOTOR_CONTROLLER_ID, MotorType.kBrushless);
  public CANSparkMax mc_leftRear = new CANSparkMax(Constants.RL_MOTOR_CONTROLLER_ID, MotorType.kBrushless);
  public MotorControllerGroup mcg_left = new MotorControllerGroup(mc_leftFront, mc_leftRear);
  // Motor controllers Right
  public CANSparkMax mc_rightFront = new CANSparkMax(Constants.FR_MOTOR_CONTROLLER_ID, MotorType.kBrushless);
  public CANSparkMax mc_rightRear = new CANSparkMax(Constants.RR_MOTOR_CONTROLLER_ID, MotorType.kBrushless);
  public MotorControllerGroup mcg_right = new MotorControllerGroup(mc_rightFront, mc_rightRear);
  
  public DifferentialDrive drivetrain = new DifferentialDrive(mcg_left, mcg_right);
  private final SparkMaxRelativeEncoder m_leftEncoder = new SparkMaxRelativeEncoder();
  private final SparkMaxRelativeEncoder m_rightEncoder = new SparkMaxRelativeEncoder();
  public final ADXRS450_Gyro gyro;
  public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(Constants.kTrackwidthMeters);
  public Rotation2d gyroAngle = new Rotation2d(0);
  public DifferentialDrive m_odometry = new DifferentialDrive(gyroAngle);
  public Pose2d m_pose;
  
  /** Creates a new DriveSubsystem. 
   * @return */
  public void DriveSubsystem() {
    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);
    
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
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  // Gets the average distance of the two encoders.
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
  }

  public SparkMaxRelativeEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  public SparkMaxRelativeEncoder getRightEncoder() {
    return m_rightEncoder;
  }
  
  public void manualDrive(double getLeftStick, double getRightStick) {
    drivetrain.tankDrive(getLeftStick, getRightStick);
  }
}
