// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
  public boolean m_isActive = true;
  static Intake m_Instance = null;
  private WPI_TalonSRX m_intakeMotor;
  private Solenoid m_leftIntakeSolenoidDeploy, m_rightIntakeSolenoidDeploy;
  private Solenoid m_leftIntakeSolenoidRetract, m_rightIntakeSolenoidRetract;
  private int m_mode = Constants.IntakeOFF_MODE;

  public Intake() {
    if (m_isActive == false) {
      return;
    }
    m_intakeMotor = new WPI_TalonSRX(Constants.IntakeMotorCAN_Address);
    m_leftIntakeSolenoidDeploy = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.LeftIntakeChannelDeploy);
    m_rightIntakeSolenoidDeploy = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.RightIntakeChannelDeploy);
    m_leftIntakeSolenoidRetract = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.LeftIntakeChannelRetract);
    m_rightIntakeSolenoidRetract = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.RightIntakeChannelRetract);
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

  public void deployIntake() {
    m_leftIntakeSolenoidRetract.set(false);
    m_rightIntakeSolenoidRetract.set(false);
    m_leftIntakeSolenoidDeploy.set(true);
    m_rightIntakeSolenoidDeploy.set(true);
  }

  public void retractIntake() {
    m_leftIntakeSolenoidDeploy.set(false);
    m_rightIntakeSolenoidDeploy.set(false);
    m_leftIntakeSolenoidRetract.set(true);
    m_rightIntakeSolenoidRetract.set(true);
  }

  public void setIntakeMotorLevel(double mLevel) {
    m_intakeMotor.set(mLevel);
    Robot.getStorage().updateStorageMotors();
  }

  public double getIntakeMotorLevel() {
    return m_intakeMotor.get();
  }

  public void runIntakeForward() {
    m_mode = Constants.IntakeIN_MODE;
    setIntakeMotorLevel(Constants.IntakeMotorLevel);
    Robot.getKicker().runKickerBackwardSlow();
  }

  public void runIntakeBackward() {
    m_mode = Constants.IntakeOUT_MODE;
    setIntakeMotorLevel(Constants.IntakeBackMotorLevel);
    Robot.getKicker().runKickerBackwardFast();
    Robot.getShooter().runShooterBackward();
  }

  public void stopIntake() {
    m_mode = Constants.IntakeOFF_MODE;
    setIntakeMotorLevel(0);
    Robot.getKicker().stopKicker();
  }

  public int getMode() {
    return m_mode;
  }

  public void ToggleIntakeForward() {
    if (m_mode != Constants.IntakeIN_MODE) {
      runIntakeForward();
    }
    else {
      stopIntake();
    }
  }

  public void ToggleIntakeBackward() {
    if (m_mode != Constants.IntakeOUT_MODE) {
      runIntakeBackward();
    }
    else {
      stopIntake();
    }
  }
  
  //  0 = no
  //  1 = yes (forwards)
  //  2 = yes (backwards)
  public int isIntakeRunning() {
    double intakeML = getIntakeMotorLevel();

    if (intakeML > 0) {
      return 2;
    }

    else if (intakeML < 0) {
      return 1;
    }

    return 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("IntakeMotorLevel", getIntakeMotorLevel());
  }
}
