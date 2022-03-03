package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ShooterSubsystem extends SubsystemBase {
  public static Spark shooterMotor1 = new Spark(Constants.shooterMotor1Channel);
  public static Spark shooterMotor2 = new Spark(Constants.shooterMotor2Channel);
    
  public static void shooter() {
    double[] motorSpeeds = {.1, .15, .2, .25, .3, .35, .4, .5, .6, .7, .8, 1};
    
    if (RobotContainer.operatorStick.getRawButton(2)) {
      for (int i = 0; i < motorSpeeds.length; i++) {
        
        shooterMotor1.set(i);
      }
    }
  }
}
