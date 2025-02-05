package org.frc6423.frc2025.util.swerveUtil;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

public abstract class SwerveConfig {

    public abstract double getMaxLinearSpeedMetersPerSec();

    public abstract double getMaxLinearAccelMetersPerSecSqrd();

    public double getMaxAngularSpeedRadsPerSec() {
        double drivebaseRadius = Math.hypot(getTrackWidthX() / 2.0, getTrackWidthY() / 2.0);
        return getMaxLinearSpeedMetersPerSec() / drivebaseRadius;
    }

    public double getMaxAngularSpeedRadsPerSecSqrd() {
        double drivebaseRadius = Math.hypot(getTrackWidthX() / 2.0, getTrackWidthY() / 2.0);
        return getMaxLinearAccelMetersPerSecSqrd()/drivebaseRadius;
    }

    public abstract double getRobotMass();

    public abstract double getRobotWidth();

    public abstract double getRobotLength();

    public abstract double getTrackWidthY();

    public abstract double getTrackWidthX();

    public abstract double getBumperWidth();

    public abstract double getBumperLength();
    
    public abstract double getWheelRadius();

    public abstract double getPivotReduction();

    public abstract double getDriveReduction();

    public double getPivotCurrentLimit() {
        return 40.0;
    }

    public double getDriveCurrentLimit() {
        return 40.0;
    }

    public abstract PIDController getRotationalFeedback();

    public abstract PIDController getTranslationFeedback();
    
    public abstract Translation2d[] getModuleLocs();

    public abstract ModuleConfig[] getModuleConfigs();

    public abstract CANcoderConfiguration getCANcoderConfig();
    
    public abstract TalonFXConfiguration getPivotConfig();

    public abstract TalonFXConfiguration getDriveConfig();
}
