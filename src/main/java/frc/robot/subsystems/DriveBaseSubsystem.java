// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


/** Add your docs here. */
public class DriveBaseSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // Motor controllers Left
  public CANSparkMax mc_leftFront = new CANSparkMax(Constants.FL_MOTOR_CONTROLLER_ID, MotorType.kBrushless);
  public CANSparkMax mc_leftRear = new CANSparkMax(Constants.RL_MOTOR_CONTROLLER_ID, MotorType.kBrushless);
  MotorControllerGroup mcg_left = new MotorControllerGroup(mc_leftFront, mc_leftRear);
  // Motor controllers Right
  public CANSparkMax mc_rightFront = new CANSparkMax(Constants.FR_MOTOR_CONTROLLER_ID, MotorType.kBrushless);
  public CANSparkMax mc_rightRear = new CANSparkMax(Constants.RR_MOTOR_CONTROLLER_ID, MotorType.kBrushless);
  MotorControllerGroup mcg_right = new MotorControllerGroup(mc_rightFront, mc_rightRear);

  // instantiate a new differential drive object
  public DifferentialDrive drivetrain = new DifferentialDrive(mcg_left, mcg_right);

  public void manualDrive(double getLeftStick, double getRightStick) {
    drivetrain.tankDrive(getLeftStick, getRightStick);
  }

  @Override
  public void periodic() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
