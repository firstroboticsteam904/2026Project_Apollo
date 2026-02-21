// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.ResetMode;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestMotor extends SubsystemBase {

  //new instance of SparkMax controlling a NEO motor with ID of 25
  private final SparkMax Testmotor = new SparkMax(25, DriveConstants.NEO);

  /** Creates a new testMotor. */
  public TestMotor() {
    //set config settings
    Testmotor.configure(Constants.kFortyAmp, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //create funtion to set motor speed
  public void speed(double power) {
    //setting motor speed based on what is inputed from the command
    Testmotor.set(power);
  }

}
