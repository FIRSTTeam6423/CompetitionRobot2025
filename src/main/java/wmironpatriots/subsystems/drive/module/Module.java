package wmironpatriots.subsystems.drive.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import wmironpatriots.subsystems.drive.module.ModuleHardware.LoggableState;

public class Module {
  /**
   * Represents the constants of a single module
   *
   * @param index Module identifier
   * @param pivotId Pivot motor CAN ID
   * @param driveId Drive motor CAN ID
   * @param encoderId Encoder CAN/PWM ID
   * @param encoderOffsetRevs Encoder measurement offset in Revs
   * @param pivotInverted Is pivot motor inverted?
   * @param driveInverted Is drive motor inverted?
   */
  public static record ModuleConfig(
      int index,
      int pivotId,
      int driveId,
      int encoderId,
      double encoderOffsetRevs,
      boolean pivotInverted,
      boolean driveInverted) {}  

  private final ModuleHardware hardware;

  public LoggableState loggableState =
      new LoggableState(false, 0, 0, 0, 0, 0, false, 0, 0, 0, 0, 0, 0, false, 0);

  public Module(ModuleHardware hardware) {
    this.hardware = hardware;
  }

  /** Periodic logic of Swerve Module */
  public void periodic() {
    loggableState = hardware.getLoggableState();
  }

  /**
   * Set angle and speed setpoints of the module
   *
   * @param setpointState {@link SwerveModuleState} representing the desired angle and speed of the
   *     module
   * @return {@link SwerveModuleState} representing the applied optimized setpoints
   */
  public SwerveModuleState setSetpoints(SwerveModuleState setpointState) {
    // Minimize the change in heading
    setpointState.optimize(getRotation2d());

    // Decrease drive speed based on distance to angle setpoint (reduces thread wear)
    setpointState.speedMetersPerSecond *= setpointState.angle.minus(getRotation2d()).getCos();

    hardware.setDriveSetpointSpeed(setpointState.speedMetersPerSecond);
    hardware.setPivotSetpointPose(setpointState.angle.getRotations());

    return setpointState;
  }

  /** Stop all module movement */
  public void stop() {
    hardware.stop();
  }

  /**
   * @return {@link Rotation2d} representing the angle of the module
   */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromRotations(loggableState.cancoderRevs());
  }

  /**
   * @return {@link SwerveModuleState} representing the measured position and speed of the module
   */
  public SwerveModuleState getSwerveModuleState() {
    return new SwerveModuleState(loggableState.driveSetpointMps(), getRotation2d());
  }

  /**
   * @return {@link SwerveModulePosition} representing the measured field pose of the module
   */
  public SwerveModulePosition getSwerveModulePosition() {
    return new SwerveModulePosition(loggableState.drivePose(), getRotation2d());
  }
}