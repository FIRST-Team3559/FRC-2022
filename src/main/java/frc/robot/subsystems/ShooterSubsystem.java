package frc.robot.subsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  public CANSparkMax shooterMotor1 = new CANSparkMax(Constants.shooterMotor1_ID, MotorType.kBrushless);
  public CANSparkMax shooterMotor2 = new CANSparkMax(Constants.shooterMotor2_ID, MotorType.kBrushless);
  
  @Override
  public void periodic() {
  
  }
}
