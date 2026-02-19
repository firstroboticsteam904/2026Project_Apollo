// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  /*
   * this file will hold all code for shooting, rotation, hood flick, and general tower stuff
   * 
   * Check limelight docs for tracking tX and converting that to motor input via PID:
   * https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltags
   * We have done similar things in code before, so also check preious years code on github.
   * 
   * Need a way to control hood angle based on distance from the apriltag with PID, how to do this can
   * also be found in the limelight docs.
   * 
   * Turret cannot rotate past 360 degrees when tracking. When it hits 359, it needs to rotate
   * back around the other direction with PID. 
   * 
   * Should keep track of ID's based on alliance color, and field location.
   * 
   * Ability to track if motor is stalled or not, if stalled, run motor in reverse to clear
   * shooter jam.
   * 
   * Shooter motors should be able to spin and stop spinning.
   */
  /** Creates a new Turret. */
  public Turret() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
