// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveBase;

public class BackOff extends CommandBase {
  /** Creates a new BackOff. */
  private DriveBase m_drivetrain;
  private double m_leftDistance, m_rightDistance;
  public BackOff() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = Robot.getDriveBase();
    addRequirements(m_drivetrain);
    m_leftDistance = 0;
    m_rightDistance = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.setLeftMotorLevel(0.7);
		m_drivetrain.setRightMotorLevel(0.7);
		m_leftDistance = m_drivetrain.getLeftDistanceInches();
		m_rightDistance = m_drivetrain.getRightDistanceInches();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.setLeftMotorLevel(0.7);
		m_drivetrain.setRightMotorLevel(0.7);
		m_leftDistance = m_drivetrain.getLeftDistanceInches();
		m_rightDistance = m_drivetrain.getRightDistanceInches();
  }
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setLeftMotorLevel(0);
		m_drivetrain.setRightMotorLevel(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((m_leftDistance + m_rightDistance) / 2) > 60;
  }
}
