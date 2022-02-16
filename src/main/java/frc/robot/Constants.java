/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //CAN Addresses
    public static final int LeftSpark1CAN_Address = 1; //TBD
    public static final int LeftSpark11CAN_Address = 11; //TBD
    public static final int RightSpark2CAN_Address = 2; //TBD
    public static final int RightSpark21CAN_Address = 21; //TBD
    public static final int RightShooterMotorCAN_Address = 45;
    public static final int LeftShooterMotorCAN_Address = 46;
    public static final int StorageCAN_Address = 7;
    public static final int IntakeMotorCAN_Address = 4;

    //PDP Ports
    public static final int RightSpark2PDP_Port = 0; //TBD
    public static final int RightSpark21PDP_Port = 1; //TBD
    public static final int LeftSpark11PDP_Port = 14; //TBD
    public static final int LeftSpark1PDP_Port = 15; //TBD
    public static final int LeftShooterMotorPDP_Port = 13;
    public static final int RightShooterMotorPDP_Port = 2;
    public static final int StoragePDP_Port = 10;
    public static final int IntakeMotorPDP_Port = 3;
    
    //Motor Levels
    public static double IntakeMotorLevel = -0.85;
    public static double IntakeBackMotorLevel = 0.5;
    public static double ShooterMotorLevel = 0; //TBD
    public static double StorageMotorLevel = 0.7;
    public static double StorageBackMotorLevel = -0.4;
    
    //Motor Stall Currents
    
    //Encoder Tics
    public static final int DriveBaseEncoderTics = 4096; //TBD
    
    //Wheel Diameters
    public static final double DriveBaseWheelDiameter = 0; //Inches //TBD
    public static final double ShooterWheelDiameter = 0; //Inches //TBD
    
    //Degrees
    //public static final double SystemDegrees = 0; //TBD

    //Tolerance
    //public static final double SystemTolerance = 20; //TBD

    //PID
    public static final double IntakeArmKp = 0.0125;

    public static final double SlowDownOffset = 0; //TBD
    public static final double SlowBaseLevel = 0; //TBD
    public static final double SlowKp = 0; //TBD
    public static final double FastVelocity = 0; //TBD
    public static final double FastBaseLevel = 0; //TBD
    public static final double FastKp = 0; //TBD
}