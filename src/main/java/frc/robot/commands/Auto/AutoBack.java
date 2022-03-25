/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveBase;
import frc.robot.util.*;

public class AutoBack extends CommandBase {
  DriveBase m_drivebase;
  int setDistance;
  double m_heading;
  double rightMotorLevelChange, leftMotorLevelChange;
  double leftInches, rightInches, averageInches;
  double startLeftInches, startRightInches;
  boolean first;
  /**
   * Creates a new AutoMoveFoward.
   */
  public AutoBack(int distance, double heading) {
    // Use addRequirements() here to declare subsystem dependencies.
    setDistance = distance;
    m_heading = heading;
    m_drivebase = Robot.getDriveBase();
    addRequirements(m_drivebase);
    rightMotorLevelChange = 0.3;
    leftMotorLevelChange = 0.3;
    leftInches = 0;
    rightInches = 0;
    startLeftInches = 0;
    startRightInches = 0;
    averageInches = 0;
    first = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Move " + setDistance + " S T A R T");
    m_drivebase.setSafetyEnabled(false);
    m_drivebase.zeroDriveSensors(true);
    System.out.println("Move " + ((m_drivebase.getLeftDistanceInches() + m_drivebase.getRightDistanceInches()) / 2) + " Z E R O");
    m_drivebase.setRightMotorLevel(-0.3);
    m_drivebase.setLeftMotorLevel(-0.3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (first) {
      return;
    }
    leftInches = m_drivebase.getLeftDistanceInches();
    rightInches = m_drivebase.getRightDistanceInches();
    averageInches = (leftInches + rightInches) / 2;
    double delta = Gyro.getYaw() - m_heading;

    if (delta > 180) {
      delta -= 360;
    }

    if (delta < -180) {
      delta += 360;
    }

    if (Math.abs(delta) > 2) {
      if (delta < 0) {
        rightMotorLevelChange -= 0.01;
        leftMotorLevelChange += 0.01;
        m_drivebase.setRightMotorLevel(-1*rightMotorLevelChange);
        m_drivebase.setLeftMotorLevel(-1*leftMotorLevelChange);
      }
      else if (delta > 0) {
        rightMotorLevelChange += 0.01;
        leftMotorLevelChange -= 0.01;
        m_drivebase.setRightMotorLevel(-1*rightMotorLevelChange);
        m_drivebase.setLeftMotorLevel(-1*leftMotorLevelChange);
      }
    }
    
    else {
      rightMotorLevelChange = 0.3;
      leftMotorLevelChange = 0.3;
      m_drivebase.setRightMotorLevel(-1*rightMotorLevelChange);
      m_drivebase.setLeftMotorLevel(-1*leftMotorLevelChange);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Move " + averageInches + " D O N E");
    m_drivebase.setLeftMotorLevel(0);
    m_drivebase.setRightMotorLevel(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (first) {
      first = false;
      return false;
    }
    if (Robot.getOI().getDrive() > 0.7) {
      return true;
    }

    if (Math.abs(averageInches) >= Math.abs(setDistance)) {
      return true;
    }

    return false;
  }
}
