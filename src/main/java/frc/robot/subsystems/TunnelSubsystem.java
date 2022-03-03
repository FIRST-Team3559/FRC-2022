package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class TunnelSubsystem extends SubsystemBase {
    public static Spark ballTunnelMotor = new Spark(Constants.ballTunnelChannel);
  
  public static void tunnel() {
    if (RobotContainer.operatorStick.getRawButton(3)) {
      ballTunnelMotor.set(-.7);
    } else {
      ballTunnelMotor.set(0);
    }
  }
}
