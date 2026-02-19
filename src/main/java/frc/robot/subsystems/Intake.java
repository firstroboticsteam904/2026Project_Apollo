// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /*
   * This will hold the code for both intakes, as well as the hopper
   * and the belt system running below the robot.
   * 
   * Intakes should ONLY ever be able to deploy one side at a time.
   * If one intake is out, the other NEEDS to be in.
   * 
   * Belt under robot should be able to run without deployment of intakes
   */
  /** Creates a new Intake. */
  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
