// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class FrontClimbUnspool extends CommandBase {
  /** Creates a new FrontClimbUnspool. */
  private Climber m_climber;
  private double startRightRotations, startLeftRotations, currentRightRotations, currentLeftRotations;
  private double totalLeftRotations, totalRightRotations;
  private double targetRotations;

  public FrontClimbUnspool(int rotations) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climber);
    targetRotations = rotations;
    startLeftRotations = m_climber.getLeftRotaions();
    startRightRotations = m_climber.getRightRotations();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.winchDown(); //i think
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentLeftRotations = m_climber.getLeftRotaions();
    currentRightRotations = m_climber.getRightRotations();
    totalLeftRotations = currentLeftRotations - startLeftRotations;
    totalRightRotations = currentRightRotations - startRightRotations;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.winchStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(totalLeftRotations) > targetRotations && Math.abs(totalRightRotations) > targetRotations) {
      return true;
    }
    return false;
  }
}
