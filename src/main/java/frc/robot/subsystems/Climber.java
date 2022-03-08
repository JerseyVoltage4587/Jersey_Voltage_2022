// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public boolean m_isActive = true;
  static Shooter m_Instance = null;
  private static WPI_TalonSRX m_leftClimberMotor;
  private static WPI_TalonSRX m_rightClimberMotor;
  private Solenoid m_leftClimberSolenoid;
  private Solenoid m_rightClimberSolenoid;
  private int numRotations;
  public Climber(int rotations) {
    if (m_isActive == false) {
      return;
    }
    numRotations = rotations;
    m_leftClimberSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.LeftClimberChannel);
    m_rightClimberSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.RightClimberChannel);
  }

  public static Shooter getInstance() {
    if(m_Instance == null) {
			synchronized (Shooter.class) {
				m_Instance = new Shooter();
			}
		}
		return m_Instance;
  }

  //Unspool winches by set amount
  public void climbStep1(double rotations) {
    //m_leftClimberMotor
    //m_rightClimberMotor
  }

  //Deploy pistons to lock
  public void climbStep2() {
    m_leftClimberSolenoid.set(true);
    m_rightClimberSolenoid.set(true);
  }

  //Reel in the winches
  public void climbStep3() {
    
  }

  //Retract pistons
  public void climbStep4() {
    m_leftClimberSolenoid.set(false);
    m_rightClimberSolenoid.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
