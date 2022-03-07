// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public boolean m_isActive = true;
  static Shooter m_Instance = null;
  private static WPI_TalonSRX m_leftClimberMotor = null;
  private static WPI_TalonSRX m_rightClimberMotor = null;
  public Climber() {
    if (m_isActive == false) {
      return;
    }
  }

  public static Shooter getInstance() {
    if(m_Instance == null) {
			synchronized (Shooter.class) {
				m_Instance = new Shooter();
			}
		}
		return m_Instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
