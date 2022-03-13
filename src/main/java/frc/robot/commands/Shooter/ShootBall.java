// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;

public class ShootBall extends CommandBase {
  private Kicker m_kicker;
  private Storage m_storage;
  
  public ShootBall() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_kicker = Robot.getKicker();
    m_storage = Robot.getStorage();
    addRequirements(m_kicker, m_storage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_kicker.runKickerForward();
    m_storage.runStorageForward();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
