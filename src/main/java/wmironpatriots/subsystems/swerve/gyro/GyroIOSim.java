package wmironpatriots.subsystems.swerve.gyro;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.geometry.Rotation2d;

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
