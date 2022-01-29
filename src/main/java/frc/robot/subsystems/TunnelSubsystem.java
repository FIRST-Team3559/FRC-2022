package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TunnelSubsystem extends SubsystemBase {
  public Spark bottomTunnelMotor = new Spark(Constants.bottomTunnelChannel);
  public Spark topTunnelMotor = new Spark(Constants.topTunnelChannel);
  
 @Override 
 public void periodic() {
   if()
     bottomTunnelMotor.enableDeadbandElimination(true);
     topTunnelMotor.enableDeadbandElimination(true);
 }
}
