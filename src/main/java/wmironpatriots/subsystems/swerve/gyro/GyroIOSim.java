// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim extends Gyro {
  private final GyroSimulation simulation;

  public GyroIOSim(GyroSimulation simulation) {
    this.simulation = simulation;
  }

  @Override
  public void periodic() {
    headingDegrees = simulation.getGyroReading().getDegrees();
  }

  @Override
  public Rotation2d getRotation2d() {
    return simulation.getGyroReading();
  }
}
