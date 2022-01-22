// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Storage extends SubsystemBase {
  public boolean m_isActive = true;
  static Storage m_Instance = null;
  public WPI_TalonSRX m_storageMotor;

  public Storage() {
    if (m_isActive == false) {
      return;
    }
    m_storageMotor = new WPI_TalonSRX(Constants.StorageCAN_Address);
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

  //  0 = no
  //  1 = yes (forwards)
  //  2 = yes (backwards)
  public int isIntakeRunning() {
    double intakeML = Robot.getIntake().getIntakeMotorLevel();

    if (intakeML > 0) {
      return 2;
    }

    else if (intakeML < 0) {
      return 1;
    }

    return 0;
  }

  public void updateStorageMotors() {
    int intakeRunning = isIntakeRunning();

    if (intakeRunning == 2) {
      setStorageMotorLevel(Constants.StorageBackMotorLevel);
    }

    else if (intakeRunning == 1) {
      setStorageMotorLevel(Constants.StorageMotorLevel);
    }

    else {
      setStorageMotorLevel(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
