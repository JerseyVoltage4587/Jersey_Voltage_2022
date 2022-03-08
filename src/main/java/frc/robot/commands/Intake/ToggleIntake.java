// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;

public class ToggleIntake extends CommandBase {
  /** Creates a new ToggleIntake. */
  public ToggleIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.getIntake(), Robot.getStorage(), Robot.getKicker());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandScheduler.getInstance().schedule(new IntakeBall());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CommandScheduler.getInstance().schedule(new StopIntakeBall());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}