// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {


  WPI_TalonFX[] motors = new WPI_TalonFX[]{
    new WPI_TalonFX(30)
  };
  
  double[] motorTotalCurrents = new double[motors.length];
  double[] motorTotalRPMs = new double[motors.length];

  DoubleLogEntry[] motorCurrents = new DoubleLogEntry[motors.length];
  DoubleLogEntry[] motorTemps = new DoubleLogEntry[motors.length];
  DoubleLogEntry[] motorRPMs = new DoubleLogEntry[motors.length];

  long count = 0;


  @Override
  public void robotInit() {

    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLog log = DataLogManager.getLog();
    
    for (int i = 0; i < motors.length; i++) {
      motorCurrents[i] = new DoubleLogEntry(log, "currents/" + motors[i].getDeviceID());
      motorTemps[i] = new DoubleLogEntry(log, "temps/" + motors[i].getDeviceID());
      motorRPMs[i] = new DoubleLogEntry(log, "rpms/" + motors[i].getDeviceID());
    }
  }

  @Override
  public void robotPeriodic() {
    for (int i = 0; i < motors.length; i++) {
      motorCurrents[i].append(motors[i].getStatorCurrent());
      motorTemps[i].append(motors[i].getTemperature());
      motorRPMs[i].append(motors[i].getSelectedSensorVelocity() / 2048 * 10 * 60);
    }
  }

  @Override
  public void teleopPeriodic() {
    for (WPI_TalonFX motor: motors) {
      motor.set(1);
    }

    if (motors[0].getStatorCurrent() < 4) {
      for (int i=0; i<motors.length; i++) {
        motorTotalCurrents[i] += motors[i].getStatorCurrent();
        motorTotalRPMs[i] += motors[i].getSelectedSensorVelocity() / 2048 * 10 * 60;
      }
      count++;
    }

  }

  @Override
  public void disabledInit() {
    for (int i=0; i<motors.length; i++) {
      System.out.println("Motor " + motors[i].getDeviceID() + " Avg Current: " + (motorTotalCurrents[i]/count)); 
      System.out.println("Motor " + motors[i].getDeviceID() + " Avg RPM: " + (motorTotalRPMs[i]/count));
    }
  }
}
