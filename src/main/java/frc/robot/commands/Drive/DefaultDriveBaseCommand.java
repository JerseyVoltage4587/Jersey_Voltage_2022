/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.DriveBase;

public class DefaultDriveBaseCommand extends CommandBase {
  /**
   * Creates a new DefaultDriveBase.
   */
  private DriveBase m_drivebase;
  private OI m_OI;

  public DefaultDriveBaseCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivebase = Robot.getDriveBase();
    m_OI = Robot.getOI();
    addRequirements(m_drivebase);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivebase.setSafetyEnabled(true);
  }

  public double motorLevelToVoltage(double motorLevel) {
    return motorLevel * RobotController.getBatteryVoltage();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forward = m_OI.getDrive();
    double turn = m_OI.getTurn();

    if (Math.abs(forward) < 0.05) {
			forward = 0;
		}
		if (Math.abs(turn) < 0.05) {
			turn = 0;
    }

    m_drivebase.arcadeDrive(Math.signum(forward) * Math.pow(forward, 2),turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.setSafetyEnabled(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}