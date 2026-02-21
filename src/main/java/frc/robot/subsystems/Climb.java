// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class Climb extends SubsystemBase {
  /*
   * This should contain the code for the climbing system.
   * 
   * Ablity to raise the climber
   * 
   * Ability to lower the climber
   * 
   * Ability to pivot arms in and out
   * 
   * One motor controls the Up/Down, One Motor controls the In/Out. Total of 2 Motors - Complete
   */

    //Motors for Climb, ID's and Motor Type assigned
    private final SparkMax kClimbPiv = new SparkMax(19, DriveConstants.NEO);
    private final SparkMax kClimbLift = new SparkMax(20, DriveConstants.NEO);



  /** Creates a new Climb. */
  public Climb() {
    //Configurations for climb motors
    kClimbPiv.configure(Constants.kThirtyAmp, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    kClimbLift.configure(Constants.kFortyAmp, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
