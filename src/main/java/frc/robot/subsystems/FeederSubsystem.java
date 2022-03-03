package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class FeederSubsystem {
  public static Spark feederMotor = new Spark (Constants.feederChannel);
  
  public static void feeder() {
    if (RobotContainer.operatorStick.getRawButton(5)) {
      feederMotor.set(0.5);
    } else {
    if (RobotContainer.operatorStick.getRawButton(4)) {
      feederMotor.set(-.5);
    } else {
      feederMotor.set(0);
    }
  }
}
}
