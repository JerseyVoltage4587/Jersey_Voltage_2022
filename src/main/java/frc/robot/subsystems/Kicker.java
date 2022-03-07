// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Kicker extends SubsystemBase {
  public boolean m_isActive = true;
  static Kicker m_Instance = null;
  private WPI_VictorSPX m_kickerMotor;

  public Kicker() {
    if (m_isActive == false) {
      return;
    }
    m_kickerMotor = new WPI_VictorSPX(Constants.KickerMotorCAN_Address);
    m_kickerMotor.configFactoryDefault();
  }

  public static Kicker getInstance() {
    if(m_Instance == null) {
			synchronized (Kicker.class) {
        if(m_Instance == null) {
          m_Instance = new Kicker();
        }
			}
		}
		return m_Instance;
  }

  public void setKickerMotorLevel(double mL) {
    m_kickerMotor.set(mL);
  }

  public double getKickerMotorLevel() {
    return m_kickerMotor.get();
  }

  public void runKickerForward() {
    setKickerMotorLevel(Constants.KickerMotorLevel);
  }

  public void runKickerBackwardSlow() {
    setKickerMotorLevel(Constants.KickerSlowBackMotorLevel);
  }

  public void runKickerBackwardFast() {
    setKickerMotorLevel(Constants.KickerFastBackMotorLevel);
  }

  public void stopKicker() {
    setKickerMotorLevel(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("KickerMotorLevel", getKickerMotorLevel());
  }
}
