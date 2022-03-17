/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.Auto.SimpleAuto;
import frc.robot.commands.Climber.ToggleClimbMotors;
import frc.robot.commands.Climber.ToggleClimbPistons;
import frc.robot.commands.Climber.WinchDown;
import frc.robot.commands.Climber.WinchStop;
import frc.robot.commands.Climber.WinchUp;
import frc.robot.commands.Intake.*;
import frc.robot.commands.Shooter.PrepareShooter;
import frc.robot.commands.Shooter.PrepareShooterHigh;
import frc.robot.commands.Shooter.ShootBall;
import frc.robot.commands.Shooter.StopShootBall;
import frc.robot.commands.Shooter.StopShooter;
import frc.robot.commands.Shooter.ToggleShooter;

public class OI extends CommandBase {
  DifferentialDrive m_drive;
  static OI m_Instance = null;
  private Joystick m_joy1 = null;
  private Joystick m_joy2 = null;
  Button buttonA1, buttonB1, buttonX1, buttonY1, 
    leftBumper1, rightBumper1, backButton1, startButton1, 
    leftStickButton1, rightStickButton1;
  JoyButton leftTrigger1, rightTrigger1;
  Button buttonA2, buttonB2, buttonX2, buttonY2, 
    leftBumper2, rightBumper2, backButton2, startButton2, 
    leftStickButton2, rightStickButton2;
  JoyButton leftTrigger2, rightTrigger2;

  public static OI getInstance() {
    if (m_Instance == null) {
      synchronized (OI.class) {
        m_Instance = new OI();
      }
    }
    return m_Instance;
  }

  /**
   * Creates a new OI.
   */
  public OI() {
    m_joy1 = new Joystick(0);
    buttonA1 = new JoystickButton(m_joy1, 1);
    buttonB1 = new JoystickButton(m_joy1, 2);
    buttonX1 = new JoystickButton(m_joy1, 3);
    buttonY1 = new JoystickButton(m_joy1, 4);
    leftBumper1 = new JoystickButton(m_joy1, 5);
    rightBumper1 = new JoystickButton(m_joy1, 6);
    backButton1 = new JoystickButton(m_joy1, 7);
    startButton1 = new JoystickButton(m_joy1, 8);
    leftStickButton1 = new JoystickButton(m_joy1, 9);
    rightStickButton1 = new JoystickButton(m_joy1, 10);
    leftTrigger1 = new JoyButton(m_joy1, JoyButton.JoyDir.DOWN, 2);
    rightTrigger1 = new JoyButton(m_joy1, JoyButton.JoyDir.DOWN, 3);
    
    m_joy2 = new Joystick(1);
    buttonA2 = new JoystickButton(m_joy2, 1);
    buttonB2 = new JoystickButton(m_joy2, 2);
    buttonX2 = new JoystickButton(m_joy2, 3);
    buttonY2 = new JoystickButton(m_joy2, 4);
    leftBumper2 = new JoystickButton(m_joy2, 5);
    rightBumper2 = new JoystickButton(m_joy2, 6);
    backButton2 = new JoystickButton(m_joy2, 7);
    startButton2 = new JoystickButton(m_joy2, 8);
    leftStickButton2 = new JoystickButton(m_joy2, 9);
    rightStickButton2 = new JoystickButton(m_joy2, 10);
    leftTrigger2 = new JoyButton(m_joy2, JoyButton.JoyDir.DOWN, 2);
    rightTrigger2 = new JoyButton(m_joy2, JoyButton.JoyDir.DOWN, 3);
    //upArrow2 = new POVButton(m_joy2, 0);
    //downArrow2 = new POVButton(m_joy2, 180);

    buttonA1.whenPressed(new SimpleAuto(30,0));
    buttonB1.whenPressed(new EjectBall());
    buttonB1.whenReleased(new StopEjectBall());
    //buttonY1.whenPressed();
    //startButton1.whenPressed();
    //backButton1.whenPressed();
    //leftStickButton1.whenPressed();
    //rightStickButton1.whenPressed();
    leftTrigger1.whenPressed(new IntakeBall());
    leftTrigger1.whenReleased(new StopIntakeBall());
    rightTrigger1.whenPressed(new ShootBall());
    rightTrigger1.whenReleased(new StopShootBall());
    //leftBumper1.whenPressed(new IntakeBall());
    //rightBumper1.whileHeld();

    buttonA2.whenPressed(new PrepareShooter());
    buttonA2.whenReleased(new StopShooter());
    buttonB2.whenPressed(new WinchDown());
    buttonB2.whenReleased(new WinchStop());
    buttonX2.whenPressed(new WinchUp());
    buttonX2.whenReleased(new WinchStop());
    /*
    upArrow2.whenPressed(new WinchUp());
    upArrow2.whenReleased(new WinchStop());
    downArrow2.whenPressed(new WinchDown());
    downArrow2.whenReleased(new WinchStop());
    */
    buttonY2.whenPressed(new PrepareShooterHigh());
    buttonY2.whenReleased(new StopShooter());
    //startButton2.whenPressed();
    //backButton2.whenPressed();
    //leftStickButton2.whenPressed();
    //rightStickButton2.whenPressed();
    leftTrigger2.whenPressed(new DeployIntake());
    rightTrigger2.whenPressed(new ToggleClimbPistons());
    //rightBumper2.whenPressed();
    leftBumper2.whenPressed(new RetractIntake());
  }

  // Get the value of the "drive" stick.
	public double getDrive() {
    return -1 * m_joy1.getRawAxis(1);
	}

	// Get the value of the "turn" stick.
	public double getTurn() {
    return m_joy1.getRawAxis(4);
	}
  // Get value of the "climb" stick.
  public double getLeftClimbStick() {
    return m_joy2.getRawAxis(1);
  }
  public double getRightClimbStick(){
    return m_joy2.getRawAxis(5);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
    return true;
  }
}