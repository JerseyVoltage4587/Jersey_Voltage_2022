// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.Climber;

public class DefaultClimberCommand extends CommandBase {
  private Climber m_climber;
  private OI m_OI;

  public DefaultClimberCommand() {
    m_climber = Robot.getClimber();
    m_OI = Robot.getOI();
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double climbLeft = m_OI.getLeftClimbStick();
    double climbRight = m_OI.getRightClimbStick();
    
    if (Math.abs(climbLeft)<0.05){
      climbLeft = 0;
    }
    if (Math.abs(climbRight)<0.05){
      climbRight = 0;
    }
    m_climber.setLeftFrontMotorLevel(climbLeft * 0.8);
    m_climber.setRightFrontMotorLevel(climbRight * 0.8);

    SmartDashboard.putNumber("Left Volts", m_climber.getLeftVolts());
    SmartDashboard.putNumber("Right Volts", m_climber.getRightVolts());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
