// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public final class Constants {

  public static final SparkMaxConfig kTwntyAmp = new SparkMaxConfig();
  public static final SparkMaxConfig kFortyAmp = new SparkMaxConfig();
  
    public static final class OperatorConstants{

        public static final double Deadzone = 0.15;
        public static final double limelightDeadzone = 0.05;
      }

    public static final double maximumSpeed = Units.feetToMeters(13.59);

    static{
      kFortyAmp.smartCurrentLimit(45);
      kTwntyAmp.smartCurrentLimit(25);
    
    }
}

