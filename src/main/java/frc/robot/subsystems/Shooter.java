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

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  public boolean m_isActive = true;
  static Shooter m_Instance = null;
  private static WPI_TalonSRX m_leftShooterMotor = null;
  private static WPI_TalonSRX m_rightShooterMotor = null;
  private double mLevel;
  private BangBangController shootController;
  private double m_setpoint = 0;
  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    if (m_isActive == false) {
      return;
    }
    m_leftShooterMotor = new WPI_TalonSRX(Constants.LeftShooterMotorCAN_Address);
    m_rightShooterMotor = new WPI_TalonSRX(Constants.RightShooterMotorCAN_Address);
    shootController = new BangBangController(50);
    m_leftShooterMotor.follow(m_rightShooterMotor);
    m_leftShooterMotor.setInverted(InvertType.OpposeMaster);
    m_leftShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    m_rightShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    m_leftShooterMotor.configFactoryDefault();
    m_rightShooterMotor.configFactoryDefault();
    m_rightShooterMotor.setNeutralMode(NeutralMode.Coast);
    m_leftShooterMotor.setNeutralMode(NeutralMode.Coast);
    m_leftShooterMotor.set(0);
    m_rightShooterMotor.set(0);
  }

  public final static int ON_MODE = 0;
  public final static int OFF_MODE = 1;
  public final static int BACK_MODE = 2;
  private int m_mode = OFF_MODE;

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
    return -1 * m_leftShooterMotor.getSelectedSensorPosition(0);
  }

  public double getRightShooterEncoder() {
    if (m_isActive == false) {
      return 0;
    }
    return -1 * m_rightShooterMotor.getSelectedSensorPosition(0);
  }

  public void setShooterSetpoint(double setpoint) { //Positive will shoot forward
    if (m_isActive == false) {
      m_setpoint = 0;
      return;
    }
    m_setpoint = setpoint;
  }

  public void runShooterForward() {
    //m_mode = ON_MODE;
    setShooterSetpoint(Constants.ShooterMotorRPM);
  }

  public void runShooterBackward() {
    //m_mode = BACK_MODE;
    setShooterSetpoint(Constants.ShooterBackMotorRPM);
  }

  public void stopShooter() {
    //m_mode = OFF_MODE;
    setShooterSetpoint(0);
  }

  public double getShooterMotorLevel() { //Positive will shoot forward
    if (m_isActive == false) {
      return 0;
    }
    return mLevel;
  }

  public double getShooterMotorRPM() { //Positive will shoot forward
    if (m_isActive == false) {
      return 0;
    }
    return m_rightShooterMotor.getSelectedSensorVelocity(0) * (60000.0/1024);
  }


  public void ToggleShooter() {
    if (m_mode != ON_MODE){
      runShooterForward();
    }
    else {
      stopShooter();
    }
  }

  public boolean isRPMUpToSpeed() {
    return getShooterMotorRPM() >= Math.abs((0.98 * Constants.ShooterMotorRPM));
  }

  @Override 
  public void periodic() {
    if (m_isActive == false) {
      return;
    }
    SmartDashboard.putNumber("Shooter RPM", getShooterMotorRPM());
    SmartDashboard.putNumber("Shooter Motor Level", getShooterMotorLevel());
    //SmartDashboard.putBoolean("Shooter On", m_mode == ON_MODE);
    SmartDashboard.putBoolean("Shooter Ready", isRPMUpToSpeed());
    m_rightShooterMotor.set(shootController.calculate(getShooterMotorRPM(), m_setpoint));
  }
}

