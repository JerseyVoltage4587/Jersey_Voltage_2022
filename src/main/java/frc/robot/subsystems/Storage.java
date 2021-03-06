// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Storage extends SubsystemBase {
  public boolean m_isActive = true;
  static Storage m_Instance = null;
  public WPI_VictorSPX m_storageMotor;

  public Storage() {
    if (m_isActive == false) {
      return;
    }
    m_storageMotor = new WPI_VictorSPX(Constants.StorageCAN_Address);
    m_storageMotor.configFactoryDefault();
  }

  public static Storage getInstance() {
    if(m_Instance == null) {
			synchronized (Storage.class) {
        if(m_Instance == null) {
          m_Instance = new Storage();
        }
			}
		}
		return m_Instance;
  }

  public void setStorageMotorLevel(double mLevel) {
    m_storageMotor.set(mLevel);
  }

  public double getStorageMotorLevel() {
    return m_storageMotor.get();
  }

  public void runStorageForward() {
    setStorageMotorLevel(Constants.StorageMotorLevel);
  }

  public void runStorageBackward() {
    setStorageMotorLevel(Constants.StorageBackMotorLevel);
  }
  public void runStorageShoot(){
    setStorageMotorLevel(Constants.StorageMotorLevelShooting);

  }

  public void stopStorage() {
    setStorageMotorLevel(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("StorageMotorLevel", getStorageMotorLevel());
  }
}
