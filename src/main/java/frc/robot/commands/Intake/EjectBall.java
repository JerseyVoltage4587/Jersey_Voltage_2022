// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EjectBall extends InstantCommand {
  private Kicker m_kicker;
  private Storage m_storage;
  private Shooter m_shooter;

  public EjectBall() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_kicker = Robot.getKicker();
    m_storage = Robot.getStorage();
    m_shooter = Robot.getShooter();
    addRequirements(m_kicker, m_storage, m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_kicker.runKickerBackwardFast();
    m_shooter.runShooterBackward();
    m_storage.runStorageBackward();
  }
}


