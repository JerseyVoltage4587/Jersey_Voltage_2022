// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Robot;

public class RunShooter extends CommandBase {
  double mLevel;
  /** Creates a new RunShooter. */
  public RunShooter(double motorLevel) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.getShooter());
    mLevel = motorLevel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      Robot.getShooter().setShooterMotorLevel(mLevel);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.getShooter().setShooterMotorLevel(mLevel);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
