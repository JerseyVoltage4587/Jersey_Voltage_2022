// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class EjectBall extends CommandBase {
  /** Creates a new RunIntake. */
  public EjectBall() {
    addRequirements(Robot.getIntake(), Robot.getKicker(), Robot.getStorage(), Robot.getShooter());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.getIntake().runIntakeBackward();
    Robot.getKicker().runKickerBackwardFast();
    Robot.getShooter().runShooterBackward();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}