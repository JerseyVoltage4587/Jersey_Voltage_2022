// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrepareShooter extends InstantCommand {
  Shooter m_shooter;
  double m_RPM;

  public PrepareShooter(double RPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = Robot.getShooter();
    m_RPM = RPM;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.runShooterForward(m_RPM);
  }
}
