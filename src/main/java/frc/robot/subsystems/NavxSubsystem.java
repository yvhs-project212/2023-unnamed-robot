// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase; 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.lang.Math;

public class NavxSubsystem extends SubsystemBase {
  /** Creates a new Navx. */
  
  private AHRS gyroScope;
  
  private double yaw;
  private double pitch;
  private double roll;
  private double wordLinearAccelX;
  private double wordLinearAccelY;
  private double wordLinearAccelZ;
  
  
  public NavxSubsystem() {
    gyroScope = new AHRS(SPI.Port.kMXP);
  }

  @Override
  public void periodic() {
    wordLinearAccelX = gyroScope.getWorldLinearAccelX();
    wordLinearAccelY = gyroScope.getWorldLinearAccelY();
    wordLinearAccelZ = gyroScope.getWorldLinearAccelZ();
    yaw = gyroScope.getYaw();
    pitch = gyroScope.getPitch();
    roll = gyroScope.getRoll(); 

     
    SmartDashboard.putNumber("YawValue", Math.floor(100*yaw+0.5)/100.0);
    SmartDashboard.putNumber("PitchValue", Math.floor(100*pitch+0.5)/100.0);
    SmartDashboard.putNumber("RollValue", Math.floor(100*roll+0.5)/100.0);
    //Displays value of Yaw, Pitch, and Roll 


    SmartDashboard.putNumber("getWorldLinearAccelX", Math.floor(100*wordLinearAccelX+0.5)/100.0);
    SmartDashboard.putNumber("getWorldLinearAccelY", Math.floor(100*wordLinearAccelY+0.5)/100.0);
    SmartDashboard.putNumber("getWorldLinearAccelZ", Math.floor(100*wordLinearAccelZ+0.5)/100.0);
    //Displays Accel values of X, Y, and Z

    // This method will be called once per scheduler run
  }
}
