// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.commands.Kicker.RunKicker;
import frc.robot.commands.Storage.RunStorage;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;

public class ShootBall extends CommandBase {
  private Shooter m_shooter;
  
  public ShootBall() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = Robot.getShooter();
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!m_shooter.isRPMUpToSpeed()) {
      CommandScheduler.getInstance().schedule(new PrepareShooter());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_shooter.isRPMUpToSpeed()) {
      CommandScheduler.getInstance().schedule(new RunKicker());
      CommandScheduler.getInstance().schedule(new RunStorage());
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
