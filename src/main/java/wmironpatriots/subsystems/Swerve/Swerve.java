package wmironpatriots.subsystems.Swerve;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Subsystem;
import wmironpatriots.Constants;
import wmironpatriots.Robot;
import wmironpatriots.subsystems.Swerve.module.Module;
import wmironpatriots.subsystems.Swerve.module.ModuleHardwareComp;
import lib.swerve.ChassisVelocity;

public class Swerve implements Subsystem {
  public static Swerve create() {
    var moduleConfigs = SwerveConstants.MODULE_CONFIGS;
    var modules = new Module[moduleConfigs.length];

    if (Robot.isReal()) {
      for (int i = 0; i < modules.length; i++) {
        modules[i] = new Module(new ModuleHardwareComp(moduleConfigs[i]));
      }
    } else {
      // ! PLACEHOLDER FOR SIM HARDWARE INIT
      for (int i = 0; i < modules.length; i++) {
        modules[i] = new Module(new ModuleHardwareComp(moduleConfigs[i]));
      }
    }

    return new Swerve();
  }

  private final Module[] modules;

  private final SwerveDriveKinematics kinematics = SwerveConstants.KINEMATICS;
  private final SwerveDrivePoseEstimator odometry;

  public Swerve(Module... modules) {
    this.modules = modules;

    odometry = new SwerveDrivePoseEstimator(kinematics, Rotation2d.kZero, getSwerveModulePositions(), new Pose2d());
  }

  @Override
  public void periodic() {
    odometry.update(Rotation2d.kZero, getSwerveModulePositions());

    for (Module module : modules) {
      module.periodic();
    }

    if (DriverStation.isDisabled()) {
      stop();
    }
  }

  /**
   * Converts a robot centric velocity into {@link SwerveModuleState} setpoint for each module
   * 
   * @param velocity {@link ChassisVelocity} representing desired robot centric velocity setpoint
   */
  public void setVelocitySetpoint(ChassisVelocity velocity) {
    // https://github.com/wpilibsuite/allwpilib/issues/7332
    var states = kinematics.toSwerveModuleStates(velocity.getRobotRelative());
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_LINEAR_SPEED);

    var speeds = kinematics.toChassisSpeeds(states);
    speeds = ChassisSpeeds.discretize(speeds, Constants.LOOPTIME.in(Seconds));
    
    states = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_LINEAR_SPEED);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setSetpoints(states[i]);
    }
  }
  
  /**
   * Stop all swerve modules
   */
  public void stop() {
    for (Module module : modules) {
      module.stop();
    }
  }

  /**
   * @return {@link Rotation2d} representing the chassis's yaw from last gyro reset
   */
  public Rotation2d getHeadingRotation2d() {
    return new Rotation2d();
  }

  /**
   * @return {@link ChassisVelocity} representing the measured chassis velocity derived from module states
   */
  public ChassisVelocity getVelocity() {
    var moduleStates = new SwerveModuleState[modules.length];
    for (int i = 0; i < 4; i++) {
      moduleStates[i] = modules[i].getSwerveModuleState();
    }

    return ChassisVelocity.fromRobotRelativeSpeeds(kinematics.toChassisSpeeds(moduleStates));
  }

  /**
   * @return {@link SwerveModuleState} array representing the measured speeds and angle of modules
   */
  public SwerveModuleState[] getSwerveModuleStates() {
    var moduleStates = new SwerveModuleState[modules.length];
    for (int i = 0; i < moduleStates.length; i++) {
      moduleStates[i] = modules[i].getSwerveModuleState();
    }
    
    return moduleStates;
  }

  /**
   * @return {@link SwerveModulePosition} array representing the measured drive distance and angle of modules
   */
  public SwerveModulePosition[] getSwerveModulePositions() {
    var modulePoses = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modulePoses.length; i++) {
      modulePoses[i] = modules[i].getSwerveModulePosition();
    }

    return modulePoses;
  }

}