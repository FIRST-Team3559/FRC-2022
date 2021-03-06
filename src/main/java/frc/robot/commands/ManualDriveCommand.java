// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ManualDriveCommand extends CommandBase {
  /** Creates a new ManualDriveCommand. */
  DriveSubsystem m_driveSubsystem;
  
  public ManualDriveCommand(DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
    
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  m_joystick.getPOV(
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (ManualDriveCommand.withInterrupt(true)) {
      ManualDriveCommand.cancel;
    }
  }
}
