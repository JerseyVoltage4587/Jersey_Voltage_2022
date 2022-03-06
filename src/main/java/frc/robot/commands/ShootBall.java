// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ShootBall extends CommandBase {
  /** Creates a new RunShooterForward. */
  public ShootBall() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.getShooter(), Robot.getKicker(), Robot.getStorage());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.getShooter().runShooterForward();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Robot.getShooter().isRPMUpToSpeed()) {
      Robot.getKicker().runKickerForward();
      Robot.getStorage().runStorageForward();
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.getKicker().stopKicker();
    Robot.getStorage().stopStorage();;
    Robot.getShooter().stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
