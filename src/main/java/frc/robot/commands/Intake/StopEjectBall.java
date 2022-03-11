// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StopEjectBall extends InstantCommand {
  private Shooter m_shooter;
  private Kicker m_kicker;
  private Intake m_intake;
  
  public StopEjectBall() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_kicker = Robot.getKicker();
    m_intake = Robot.getIntake();
    m_shooter = Robot.getShooter();
    addRequirements(m_kicker, m_intake, m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.stopIntake();
    m_shooter.stopShooter();
    m_kicker.stopKicker();
  }
}
