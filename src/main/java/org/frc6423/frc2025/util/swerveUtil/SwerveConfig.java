package org.frc6423.frc2025.util.swerveUtil;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

public abstract class SwerveConfig {
    // Kinematic constants
    public abstract double getMaxLinearSpeedMetersPerSec();

    public abstract double getMaxLinearAccelMetersPerSecSqrd();

    public double getMaxAngularSpeedRadsPerSec() {
        double drivebaseRadius = Math.hypot(getTrackWidthXMeters() / 2.0, getTrackWidthYMeters() / 2.0);
        return getMaxLinearSpeedMetersPerSec() / drivebaseRadius;
    }

    public double getMaxAngularSpeedRadsPerSecSqrd() {
        double drivebaseRadius = Math.hypot(getTrackWidthXMeters() / 2.0, getTrackWidthYMeters() / 2.0);
        return getMaxLinearAccelMetersPerSecSqrd()/drivebaseRadius;
    }

    // Robot characteristics
    public abstract double getRobotMassKg();

    public abstract double getRobotWidthMeters();

    public abstract double getRobotLengthMeters();

    public abstract double getTrackWidthYMeters();

    public abstract double getTrackWidthXMeters();

    public abstract double getBumperWidthInches();

    public abstract double getBumperLengthMeters();

    public abstract Translation2d[] getModuleLocs();

    // Module characteristics
    public abstract double getPivotReduction();

    public abstract double getDriveReduction();

    public double getPivotCurrentLimitAmps() {
        return 40.0;
    }

    public double getDriveCurrentLimitAmps() {
        return 40.0;
    }

    public abstract double getWheelRadiusInches();

    public abstract ModuleConfig[] getModuleConfigs();

    // Gains
    public abstract PIDController getRotationalFeedback();

    public abstract PIDController getTranslationFeedback();

    // CTRe Configs
    public abstract CANcoderConfiguration getCANcoderConfig();
    
    public abstract TalonFXConfiguration getPivotConfigTalonFX();

    public abstract TalonFXConfiguration getDriveConfigTalonFX();

    // REV Configs
    public abstract AlternateEncoderConfig getPivotABSEncoderConfig();

    public abstract SparkMaxConfig getPivotConfigSparkMax();

    public abstract SparkMaxConfig getDriveConfigSparkMax();

    // Gyro
    public abstract GyroType getGyroType();

    public abstract int getGyroID();

    public enum GyroType {
        NAVX,
        PIGEON
    }
}
