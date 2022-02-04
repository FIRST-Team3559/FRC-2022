package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class FeederSubsystem {
  public Spark feederMotor = new Spark (Constants.feederChannel);
  
  public void feeder() {
    if (driverGamepad.getRawButton(1)) {
      feederMotor.set(0.8);
    } else {
      feederMotor.set(0);
    }
  }
