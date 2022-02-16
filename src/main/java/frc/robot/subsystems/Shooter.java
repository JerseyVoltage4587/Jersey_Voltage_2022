
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  public boolean m_isActive = true;
  static Shooter m_Instance = null;
  private static CANSparkMax m_leftShooterMotor = null;
  private static CANSparkMax m_rightShooterMotor = null;
  private RelativeEncoder m_leftShooterEncoder;
  private RelativeEncoder m_rightShooterEncoder;
  private double mLevel;
  private BangBangController shootController;
  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    if (m_isActive == false) {
      return;
    }
    shootController = new BangBangController(5);
    m_leftShooterMotor = new CANSparkMax(Constants.LeftShooterMotorCAN_Address, MotorType.kBrushless);
    m_rightShooterMotor = new CANSparkMax(Constants.RightShooterMotorCAN_Address, MotorType.kBrushless);
    m_leftShooterMotor.follow(m_rightShooterMotor, /*invert=*/ true);
    m_leftShooterEncoder = m_leftShooterMotor.getEncoder();
    m_rightShooterEncoder = m_rightShooterMotor.getEncoder();
    m_leftShooterMotor.restoreFactoryDefaults();
    m_rightShooterMotor.restoreFactoryDefaults();
    m_leftShooterMotor.set(0);
    m_rightShooterMotor.set(0);
  }

  public static Shooter getInstance() {
    if(m_Instance == null) {
			synchronized (Shooter.class) {
				m_Instance = new Shooter();
			}
		}
		return m_Instance;
  }
  
  public double getLeftShooter() {
    if (m_isActive == false) {
      return 0;
    }
    return m_leftShooterMotor.get();
  }

  public double getRightShooter() {
    if (m_isActive == false) {
      return 0;
    }
    return m_rightShooterMotor.get();
  }

  public double getLeftShooterEncoder() {
    if (m_isActive == false) {
      return 0;
    }
    return m_leftShooterEncoder.getPosition();
  }

  public double getRightShooterEncoder() {
    if (m_isActive == false) {
      return 0;
    }
    return m_rightShooterEncoder.getPosition();
  }

  public void setShooterMotorLevel(double x) { //Positive will shoot forward
    if (m_isActive == false) {
      return;
    }
    mLevel = x;
    m_leftShooterMotor.set(-1 * mLevel);
    m_rightShooterMotor.set(mLevel);
  }

  public double getShooterMotorLevel() { //Positive will shoot forward
    if (m_isActive == false) {
      return 0;
    }
    return mLevel;
  }

  @Override
  public void periodic() {
    if (m_isActive == false) {
      return;
    }
    SmartDashboard.putNumber("ProcessVariable", m_rightShooterEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Motor Level", getShooterMotorLevel());
  }
}
