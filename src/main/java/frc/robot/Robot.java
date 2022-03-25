// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Auto.Auto;
import frc.robot.commands.Climber.DefaultClimberCommand;
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.commands.Drive.DefaultDriveBaseCommand;
import frc.robot.subsystems.*;
import frc.robot.util.Gyro;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */


public class Robot extends TimedRobot {
  private static RobotContainer m_robotContainer = new RobotContainer();
  private Command m_autonomousCommand;
  private static PowerDistribution m_PDP;
  private NetworkTableInstance Table;
  private NetworkTable limelight;
  private NetworkTableEntry camMode, stream;
  public static double autoWaitTime;

  public static PowerDistribution getPDP() {
    if (m_PDP == null) {
      m_PDP = new PowerDistribution();
    }
    return m_PDP;
  }

  public static DriveBase getDriveBase() {
    return DriveBase.getInstance();
  }

  public static Shooter getShooter() {
    return Shooter.getInstance();
  }

  public static Intake getIntake() {
    return Intake.getInstance();
  }

  public static Storage getStorage() {
    return Storage.getInstance();
  }

  public static Kicker getKicker() {
    return Kicker.getInstance();
  }
  
  public static Climber getClimber() {
    return Climber.getInstance();
  }

  public static Gyro getGyro() {
    return Gyro.getInstance();
  }
  
  public static OI getOI() {
    return OI.getInstance();
  }
  
  public void runLimeLight() {
    Table = NetworkTableInstance.getDefault();
    limelight = Table.getTable("limelight");
    
    camMode = limelight.getEntry("camMode");
    stream = limelight.getEntry("stream");
    
    camMode.setNumber(1);
    stream.setNumber(0);
  }
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").forceSetNumber(2);
    //runLimeLight();
    CameraServer.startAutomaticCapture();
    SmartDashboard.putNumber("Auto Wait Time", 0);
    getDriveBase().setDefaultCommand(new DefaultDriveBaseCommand());
    getClimber().setDefaultCommand(new DefaultClimberCommand());
    getDriveBase().zeroDriveSensors(true);
    CommandScheduler.getInstance().cancelAll(); //Makes sure nothing is running from a previous enable
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    // SmartDashboard.putNumber("tv", NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0));
    // SmartDashboard.putNumber("ty", NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0));
    // SmartDashboard.putNumber("tx", NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0));
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand()/*AutoChoice.getAutoChoice()*/;
    //CommandScheduler.getInstance().cancelAll(); //Makes sure nothing is running from a previous enable
    autoWaitTime = SmartDashboard.getNumber("Auto Wait Time", 0);
    CommandScheduler.getInstance().schedule(new WaitCommand(autoWaitTime) , m_autonomousCommand);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    CommandScheduler.getInstance().cancelAll(); //Makes sure nothing is running from a previous enable
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
