package org.frc6423.frc2025.subsystems.swerve.constants;

import java.util.Arrays;

import org.frc6423.frc2025.util.swerveUtil.ModuleConfig;
import org.frc6423.frc2025.util.swerveUtil.SwerveConfig;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class CompBotSwerveConfigs extends SwerveConfig {

    // Kinematics Constants
    // ! fix
    @Override
    public double getMaxLinearSpeedMetersPerSec() {
        // https://www.chiefdelphi.com/t/how-to-calculate-the-max-free-speed-of-a-swerve/400741/3
        return Units.rotationsToRadians(6000 / 60) / getDriveReduction() * getWheelRadiusInches();
    }

    @Override
    public double getMaxLinearAccelMetersPerSecSqrd() {
        return Units.feetToMeters(16.0);
    }

    // Robot characteristics
    @Override
    public double getRobotMassKg() {
        return 0.0;
    }

    @Override
    public double getRobotWidthMeters() {
        return 0.0;
    }

    @Override
    public double getRobotLengthMeters() {
        return 0.0;
    }

    @Override
    public double getTrackWidthYMeters() {
        return 0.0;
    }

    @Override
    public double getTrackWidthXMeters() {
        return 0.0;
    }

    @Override
    public double getBumperWidthInches() {
        return 0.0;
    }

    @Override
    public double getBumperLengthMeters() {
        return 0.0;
    }

    @Override
    public Translation2d[] getModuleLocs() {
        return new Translation2d[] {
            new Translation2d(0.381, 0.381),
            new Translation2d(0.381, -0.381),
            new Translation2d(-0.381, 0.381),
            new Translation2d(-0.381, -0.381)
        };
    }

    // Module characteristics
    @Override
    public double getPivotReduction() {
        return 0.0;
    }

    @Override
    public double getDriveReduction() {
        return 0.0;
    }

    @Override
    public double getWheelRadiusInches() {
        return 0.0;
    }

    @Override
    public ModuleConfig[] getModuleConfigs() {
        var configs = new ModuleConfig[] {
            new ModuleConfig(
                1, 
                1, 
                2, 
                0, 
                Rotation2d.fromRadians(0), 
                true, 
                getPivotConfigSparkMax(), 
                getDriveConfigSparkMax()),
            new ModuleConfig(
                2, 
                3, 
                4, 
                1, 
                Rotation2d.fromRadians(0), 
                true, 
                getPivotConfigSparkMax(), 
                getDriveConfigSparkMax()),
            new ModuleConfig(
                3, 
                5, 
                6, 
                2, 
                Rotation2d.fromRadians(0), 
                true, 
                getPivotConfigSparkMax(), 
                getDriveConfigSparkMax()),
            new ModuleConfig(
                4, 
                7, 
                8, 
                3, 
                Rotation2d.fromRadians(0), 
                true, 
                getPivotConfigSparkMax(), 
                getDriveConfigSparkMax()),
        };

        Arrays.stream(configs)
            .forEach((c) -> c.kWheelRadiusMeters = Units.inchesToMeters(getWheelRadiusInches())); // Shut up

        return configs;
    }

    // Gains
    @Override
    public PIDController getRotationalFeedback() {
        return new PIDController(
            0.0, 
            0.0, 
            0.0);
    }

    @Override
    public PIDController getTranslationFeedback() {
        return new PIDController(
            0.0, 
            0.0, 
            0.0);
    }

    // CTRe Configs
    @Override
    public CANcoderConfiguration getCANcoderConfig() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        return config;
    }

    @Override
    public TalonFXConfiguration getPivotConfigTalonFX() {
      TalonFXConfiguration config = new TalonFXConfiguration();

      config.Audio.BeepOnBoot = true; // boop

      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

      config.CurrentLimits.StatorCurrentLimit = getPivotCurrentLimitAmps();
      config.CurrentLimits.StatorCurrentLimitEnable = true;

      // Torque
      config.TorqueCurrent.PeakForwardTorqueCurrent = getPivotCurrentLimitAmps();
      config.TorqueCurrent.PeakReverseTorqueCurrent = -getPivotCurrentLimitAmps();
      
      // Feedback config
      config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
      config.Feedback.RotorToSensorRatio = getPivotReduction();
      config.Feedback.SensorToMechanismRatio = 1.0;
      config.Feedback.FeedbackRotorOffset = 0.0;
      config.ClosedLoopGeneral.ContinuousWrap = true; // Takes the shortest path

      // Gains
      config.Slot0.kP = 0.0;
      config.Slot0.kI = 0.0;
      config.Slot0.kD = 0.0;

      config.Slot0.kV = 0.0;
      config.Slot0.kA = 0.0;
      config.Slot0.kS = 0.0;

      config.MotionMagic.MotionMagicCruiseVelocity = 0;
      config.MotionMagic.MotionMagicAcceleration = 0;
      config.MotionMagic.MotionMagicJerk = 0;

      return config;
    }

    @Override
    public TalonFXConfiguration getDriveConfigTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        return config;
    }

    // REV Configs
    @Override
    public AlternateEncoderConfig getPivotABSEncoderConfig() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPivotABSEncoderConfig'");
    }

    @Override
    public SparkMaxConfig getPivotConfigSparkMax() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPivotConfigSparkMax'");
    }

    @Override
    public SparkMaxConfig getDriveConfigSparkMax() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getDriveConfigSparkMax'");
    }

    // Gyro
    @Override
    public GyroType getGyroType() {
        return GyroType.PIGEON;
    }

    @Override
    public int getGyroID() {
        return 0;
    }
}
