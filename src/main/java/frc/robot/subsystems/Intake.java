// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class Intake extends SubsystemBase {
  /*
   * This will hold the code for both intakes, as well as the hopper
   * and the belt system running below the robot.
   * 
   * Intakes should ONLY ever be able to deploy one side at a time.
   * If one intake is out, the other NEEDS to be in.
   * 
   * Belt under robot should be able to run without deployment of intakes
   * 
   * One Motor (per side) to pivot, one motor (per side) to pull it in. Total of 4 Motors (Two per side)
   * 
   * Belts Under the robot still TBD, but will either be one or two
   */
  /** Creates a new Intake. */

  //All Intake motors, ID's and MotorType assigned
  public final SparkMax kLTPiv = new SparkMax(9, DriveConstants.NEO);
  public final SparkMax kLTRoll = new SparkMax(10, DriveConstants.NEO);
  //public final SparkMax kRTPiv = new SparkMax(11, DriveConstants.NEO);
  //public final SparkMax kRTRoll = new SparkMax(12, DriveConstants.NEO);

  //Belly Belt ID and Motor Type assigned
  public final SparkMax kBellyBelt = new SparkMax(13, DriveConstants.NEO);

  public Intake() {
    //Configurations for all intake motors
    kLTPiv.configure(Constants.LeftIntakePivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    kLTRoll.configure(Constants.LeftIntakeRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //kRTPiv.configure(Constants.RightIntakePivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //kRTRoll.configure(Constants.RightIntakeRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //Configuration for Belly Belt Motor
    kBellyBelt.configure(Constants.BellyBeltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double LTIntakePivTicks(){
    double LTPivotTicks;
    LTPivotTicks = kLTPiv.getEncoder().getPosition();
    SmartDashboard.putNumber("LTPivTicks", LTPivotTicks);
    return LTPivotTicks;
  }

  /*public double RTIntakePivTicks(){
    double RTPivotTicks;
    RTPivotTicks = kRTPiv.getEncoder().getPosition();
    SmartDashboard.putNumber("RTPivTicks", RTPivotTicks);
    return RTPivotTicks;
  }*/

  public void LTIntakePivPower(double LTPivVolts){
    kLTPiv.setVoltage(LTPivVolts);
  }

  /*public void RTIntakePivPower(double RTPivVolts){
    kRTPiv.setVoltage(RTPivVolts);
  }*/

  public void LTIntakeRoll(double LTRollVolts){
    kLTRoll.setVoltage(LTRollVolts);
  }

  /*public void RTIntakeRoll(double RTRollVolts){
    kRTRoll.setVoltage(RTRollVolts);
  }*/

  public void BellyBeltRoll(double BellyRollVolts){
    kBellyBelt.setVoltage(BellyRollVolts);
  }

}
