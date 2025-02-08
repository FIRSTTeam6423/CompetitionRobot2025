package org.frc6423.frc2025.subsystems.swerve;

import static org.frc6423.frc2025.Constants.kTickSpeed;

import java.util.Arrays;

import org.frc6423.frc2025.subsystems.swerve.module.Module;
import org.frc6423.frc2025.util.swerveUtil.SwerveConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveConfig m_config;

    private final Module[] m_modules;

    private SwerveDriveKinematics m_kinematics;
    private SwerveDrivePoseEstimator m_poseEstimator;

    public SwerveSubsystem(SwerveConfig config) {
        // Create modules
        var moduleConfigs = config.getModuleConfigs();
        m_modules = new Module[moduleConfigs.length];
        Arrays.stream(moduleConfigs)
            .forEach((c) -> m_modules[c.kIndex] = new Module(c));

        // Create math objects
        m_kinematics = new SwerveDriveKinematics(config.getModuleLocs());
        m_poseEstimator = new SwerveDrivePoseEstimator(m_kinematics, getHeading(), getModulePoses(), new Pose2d());

        m_config = config;
    }

    @Override
    public void periodic() {
        
        // Stop module input when driverstation is disabled
        if (DriverStation.isDisabled()) {
            for (Module module : m_modules) {
                module.stop();
            }
        }
    }

    /**
     * Runs swerve at desired robot relative velocity
     * 
     * @param velocity desired velocity
     * @param openloopEnabled enable or disable open loop voltage control
     */
    public void runVelocities(ChassisSpeeds velocity, boolean openloopEnabled) {
        velocity = ChassisSpeeds.discretize(velocity, kTickSpeed);

        SwerveModuleState[] desiredStates = m_kinematics.toSwerveModuleStates(velocity);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, m_config.getMaxLinearSpeedMetersPerSec());

        for(int i = 0; i < desiredStates.length; i++) {
            if (openloopEnabled) {
                ChassisSpeeds currentVelocities = getVelocitiesRobotRelative();
                boolean focEnabled = 
                    Math.sqrt(
                        Math.pow(currentVelocities.vxMetersPerSecond, 2) 
                        + Math.pow(currentVelocities.vyMetersPerSecond, 2)) // converts linear velocity components to linear velocity
                    < 1 * m_config.getMaxLinearSpeedMetersPerSec();

                    // Converts desired motor velocity into input voltage
                    // OmegaRadsPerSec/(KvRadsPerVolt)
                    double driveVoltage = desiredStates[i].speedMetersPerSecond / DCMotor.getKrakenX60(1).KvRadPerSecPerVolt;
                    m_modules[i].runSetpointOpenloop(driveVoltage, desiredStates[i].angle, focEnabled);
            } else {
                m_modules[i].runSetpoint(desiredStates[i]);
            }
        }
    }

    /** Gets current robot velocity (robot relative) */
    public ChassisSpeeds getVelocitiesRobotRelative() {
        return m_kinematics.toChassisSpeeds(getModuleStates());
    }

    /** Gets current robot heading */
    public Rotation2d getHeading() {
        return new Rotation2d();
    }

    /** Gets current robot field pose */
    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    /** Returns an array of module field positions */
    public SwerveModulePosition[] getModulePoses() {
        return Arrays.stream(m_modules)
            .map(Module::getModulePose)
            .toArray(SwerveModulePosition[]::new);
    }

    /** Returns an array of module states */
    public SwerveModuleState[] getModuleStates() {
        return Arrays.stream(m_modules)
            .map(Module::getModuleState)
            .toArray(SwerveModuleState[]::new);
    }
}
