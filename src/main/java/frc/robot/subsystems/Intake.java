// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
  public boolean m_isActive = true;
  static Intake m_Instance = null;
  private WPI_TalonSRX m_intakeMotor;

  public Intake() {
    if (m_isActive == false) {
      return;
    }
    m_intakeMotor = new WPI_TalonSRX(Constants.IntakeMotorCAN_Address);
    m_intakeMotor.configFactoryDefault();
  }

  public static Intake getInstance() {
    if(m_Instance == null) {
			synchronized (Intake.class) {
        if(m_Instance == null) {
          m_Instance = new Intake();
        }
			}
		}
		return m_Instance;
  }

  public final static int OFF_MODE = 0;
  public final static int IN_MODE = 1;
  public final static int OUT_MODE = 2;
  private int m_mode = OFF_MODE;

  public void setIntakeMotorLevel(double mLevel) {
    m_intakeMotor.set(mLevel);
    Robot.getStorage().updateStorageMotors();
  }

  public double getIntakeMotorLevel() {
    return m_intakeMotor.get();
  }

  public void runIntakeForward() {
    m_mode = IN_MODE;
    setIntakeMotorLevel(Constants.IntakeMotorLevel);
    Robot.getKicker().runKickerBackwardSlow();
  }

  public void runIntakeBackward() {
    m_mode = OUT_MODE;
    setIntakeMotorLevel(Constants.IntakeBackMotorLevel);
    Robot.getKicker().runKickerBackwardFast();
    Robot.getShooter().runShooterBackward();
  }

  public void stopIntake() {
    m_mode = OFF_MODE;
    setIntakeMotorLevel(0);
    Robot.getKicker().stopKicker();
  }

  public int getMode() {
    return m_mode;
  }

  public void ToggleIntakeForward() {
    if(m_mode != IN_MODE) {
      runIntakeForward();
    } else {
      stopIntake();
    }
  }

  public void ToggleIntakeBackward() {
    if(m_mode != OUT_MODE) {
      runIntakeBackward();
    } else {
      stopIntake();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(
            "IntakeMotorLevel", getIntakeMotorLevel());
  }
}
