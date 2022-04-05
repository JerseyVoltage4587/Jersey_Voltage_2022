/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.AsyncStructuredLogger;
import frc.robot.util.Gyro;

public class DriveBase extends SubsystemBase {
  private boolean m_isActive = true;
  public DifferentialDrive m_drive = null;
  static DriveBase m_Instance = null;
  private CANSparkMax m_leftSpark;
  private CANSparkMax m_leftSpark11;
  private CANSparkMax m_rightSpark;
  private CANSparkMax m_rightSpark21;
  private RelativeEncoder m_leftDriveEncoder;
  private RelativeEncoder m_rightDriveEncoder;
  private double LeftMotorLevel;
  private double RightMotorLevel;
  private double partialLeftInches = 0;
  private double partialRightInches = 0;
  private DriveBaseLoggingData m_loggingData;
  private AsyncStructuredLogger<DriveBaseLoggingData> m_logger;
  private long m_lastLogTime = 0;
  private String layout = "";


  /**
   * Creates a new DriveBase.
   */
  public DriveBase() {
    if (m_isActive == false) {
      return;
    }
    m_leftSpark = new CANSparkMax(Constants.LeftSpark1CAN_Address, MotorType.kBrushless);
    m_leftSpark11 = new CANSparkMax(Constants.LeftSpark11CAN_Address, MotorType.kBrushless);
    m_rightSpark = new CANSparkMax(Constants.RightSpark2CAN_Address, MotorType.kBrushless);
    m_rightSpark21 = new CANSparkMax(Constants.RightSpark21CAN_Address, MotorType.kBrushless);
    m_leftSpark.restoreFactoryDefaults();
    m_rightSpark.restoreFactoryDefaults();
    m_leftSpark11.follow(m_leftSpark, /*invert=*/ false);
    m_rightSpark21.follow(m_rightSpark, /*invert=*/ false);
    m_leftDriveEncoder = m_leftSpark.getEncoder();
    m_rightDriveEncoder = m_rightSpark.getEncoder();
    m_leftSpark.set(0);
    m_rightSpark.set(0);
    m_leftSpark.setIdleMode(IdleMode.kBrake);
    m_rightSpark.setIdleMode(IdleMode.kBrake);
    m_leftSpark.setInverted(false);
    m_rightSpark.setInverted(true);
    m_leftSpark.setSmartCurrentLimit(30);
    m_rightSpark.setSmartCurrentLimit(30);
    m_leftSpark.setOpenLoopRampRate(0.1);
    m_rightSpark.setOpenLoopRampRate(0.1);
    m_drive = new DifferentialDrive(m_leftSpark, m_rightSpark);
    m_drive.setSafetyEnabled(false);
    m_loggingData = new DriveBaseLoggingData();
    m_logger = new AsyncStructuredLogger<DriveBaseLoggingData>("DriveBase", /*forceUnique=*/false, DriveBaseLoggingData.class);
  }

  public static DriveBase getInstance() {
    if (m_Instance == null) {
      synchronized (DriveBase.class) {
        if (m_Instance == null) {
          m_Instance = new DriveBase();
        }
      }
    }
    return m_Instance;
  }

  //Left Motor Variables
    public void setLeftVolts(double v) {
      m_leftSpark.setVoltage(v);
    }

    public double getLeftMotorLevel() {
      if (m_isActive == false) {
        return -1;
      }
      return LeftMotorLevel;
    }

    public void setLeftMotorLevel(double x) {
      if (m_isActive == false) {
        return;
      }
      LeftMotorLevel = x;
      m_leftSpark.set(LeftMotorLevel);
    }

    public double getLeftEncoder() {
      if (m_isActive == false) {
        return 0;
      }
      return m_leftDriveEncoder.getPosition() / 10.75;
    }

    public double getLeftDistanceInches() {
      return getLeftEncoder() * Constants.DriveBaseWheelDiameter * Math.PI;
    }

    public double getPartialLeftInches() {
      return partialLeftInches;
    }

  //Right Motor Variables
    public void setRightVolts(double v) {
      m_rightSpark.setVoltage(v);
    }

    public double getRightMotorLevel() {
      if (m_isActive == false) {
        return -1;
      }
      return RightMotorLevel;
    }

    public void setRightMotorLevel(double x) {
      if (m_isActive == false) {
        return;
      }
      RightMotorLevel = x;
      m_rightSpark.set(RightMotorLevel);
    }

    public double getRightEncoder() {
      if (m_isActive == false) {
        return 0;
      }
      return m_rightDriveEncoder.getPosition() / 10.75;
    }

    public double getRightDistanceInches() {
      return getRightEncoder() * Constants.DriveBaseWheelDiameter * Math.PI;
    }

    public double getPartialRightInches() {
      return partialRightInches;
    }

  public void setLayout(String l) {
    layout = l;
  }

  public String getLayout() {
    return layout;
  }

  public void setSafetyEnabled(boolean x){
    if (m_isActive == false) {
      return;
    }
    m_drive.setSafetyEnabled(x);
  }

  public void zeroDriveSensors(boolean gyro) {
    if (m_isActive == false) {
      return;
    }
    m_leftDriveEncoder.setPosition(0);
    m_rightDriveEncoder.setPosition(0);
    if (gyro) {
      Gyro.getInstance();
      Gyro.reset();
    }
  }

  public void arcadeDrive(double forward, double turn) {
    if (m_isActive == false) {
      return;
    }
    m_drive.arcadeDrive(forward, turn);
  }

  private double getRateOfChange(double initialValue, double finalValue, long initialTime, long finalTime) {
    return (finalValue - initialValue) / (finalTime - initialTime);
  }

  @Override
  public void periodic() {
    if (m_isActive == false) {
      return;
    }

    // This method will be called once per scheduler run
    long now = System.nanoTime();
    double lastLeftPosition = m_loggingData.LeftPosition;
    double lastLeftVelocity = m_loggingData.LeftVelocity;
    double lastRightPosition = m_loggingData.RightPosition;
    double lastRightVelocity = m_loggingData.RightVelocity;

    m_loggingData.LeftMotorLevel = m_leftSpark.get();
    m_loggingData.LeftMotor1_SupplyCurrent = m_leftSpark.getOutputCurrent();
    m_loggingData.LeftMotor2_SupplyCurrent = Robot.getPDP().getCurrent(Constants.LeftSpark11PDP_Port);
    m_loggingData.LeftEncoderReading = getLeftEncoder();
    m_loggingData.LeftPosition = getLeftDistanceInches();
    m_loggingData.LeftVelocity = getRateOfChange(lastLeftPosition, m_loggingData.LeftPosition, m_lastLogTime, now);
    m_loggingData.LeftAcceleration = getRateOfChange(lastLeftVelocity, m_loggingData.LeftVelocity, m_lastLogTime, now);

    m_loggingData.RightMotorLevel = m_rightSpark.get();
    m_loggingData.RightMotor1_SupplyCurrent = m_rightSpark.getOutputCurrent();
    m_loggingData.RightMotor2_SupplyCurrent = Robot.getPDP().getCurrent(Constants.RightSpark21PDP_Port);
    m_loggingData.RightEncoderReading = getRightEncoder();
    m_loggingData.RightPosition = getRightDistanceInches();
    m_loggingData.RightVelocity = getRateOfChange(lastRightPosition, m_loggingData.RightPosition, m_lastLogTime, now);
    m_loggingData.RightAcceleration = getRateOfChange(lastRightVelocity, m_loggingData.RightVelocity, m_lastLogTime, now);

    m_loggingData.Heading = Gyro.getYaw();
    m_logger.queueData(m_loggingData);
    m_lastLogTime = now;

    SmartDashboard.putNumber("Left Vel", m_loggingData.LeftEncoderReading);
    SmartDashboard.putNumber("Right Motor Level", getRightMotorLevel());
    SmartDashboard.putNumber("Left Motor Level", getLeftMotorLevel());
    SmartDashboard.putNumber("Left Encoder", getLeftEncoder());
    SmartDashboard.putNumber("Left Distance", getLeftDistanceInches());
    SmartDashboard.putNumber("Right Distance", getRightDistanceInches());
  }

  public static class DriveBaseLoggingData {
    public double LeftMotorLevel;
    public double LeftMotor1_SupplyCurrent;
    public double LeftMotor1_StatorCurrent;
    public double LeftMotor2_SupplyCurrent;
    public double LeftEncoderReading;
    public double LeftPosition;
    public double LeftVelocity;
    public double LeftAcceleration;
    public double RightMotorLevel;
    public double RightMotor1_SupplyCurrent;
    public double RightMotor1_StatorCurrent;
    public double RightMotor2_SupplyCurrent;
    public double RightEncoderReading;
    public double RightPosition;
    public double RightVelocity;
    public double RightAcceleration;
    public double Heading;
  }
}
