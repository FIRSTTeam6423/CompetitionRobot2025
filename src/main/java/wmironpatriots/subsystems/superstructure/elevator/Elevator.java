package wmironpatriots.subsystems.superstructure.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import lib.LoggedSubsystem;
import monologue.Annotations.Log;
import wmironpatriots.Robot;

public abstract class Elevator implements LoggedSubsystem {
  // * LOGGED VALUES
  @Log protected double poseRevs;
  @Log protected double setpointPose;
  @Log protected double parentCurrentAmps;
  @Log protected double parentTorqueAmps;
  @Log protected double parentAppliedVolts;
  @Log protected double parentTempCelsius;
  @Log protected double childCurrentAmps;
  @Log protected double childTorqueAmps;
  @Log protected double childAppliedVolts;
  @Log protected double childTempCelsius;

  private boolean isZeroed = false;

  public static Elevator createElevator() {
    return Robot.isReal() 
      ? new ElevatorIOComp() 
      : new ElevatorIOComp(); // ! SIMULATION PLACEHOLDER
  }

  /** Runs elevator down until current spikes above threshold */
  public Command runCurrentZeroingCmd() {
    if (isZeroed) return this.runOnce(() -> {});
    return this.run(() -> setMotorCurrent(-1.0))
      .until(() -> parentCurrentAmps > 20.0)
      .finallyDo((i) -> {
        stopMotors();
        setEncoderPose(0.0);
        System.out.println("Elevator zeroed");
        isZeroed = true;
      });
  }

  /**
   * Runs elevator to specified pose
   * 
   * @param poseRevs desired pose in revs
   */
  public Command runPoseCmd(double poseRevs) {
    return this.run(() -> {
      setpointPose = poseRevs;
      setMotorPose(poseRevs);
    });
  }

  /**
   * Checks if elevator pose is around setpoint pose
   * 
   * @return true if pose is ±0.5 from setpoint
   */
  public boolean nearSetpoint() {
    return Math.abs(setpointPose - poseRevs) > 0.5;
  }

  // * HARDWARE METHODS
  protected abstract void setMotorCurrent(double amps);
  
  protected abstract void setMotorPose(double poseRevs);

  /** Stops all elevator motor output */
  public abstract void stopMotors();

  protected abstract void setEncoderPose(double poseRevs);

  protected abstract void enableCoastMode(boolean enabled);
}