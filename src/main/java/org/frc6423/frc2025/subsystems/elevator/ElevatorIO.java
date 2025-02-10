package org.frc6423.frc2025.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    public class ElevatorIOInputs {
        public boolean LMotorEnabled = false;
        public boolean RMotorEnabled = false;

        public double poseMeters = 0.0;
        public double velMetersPerSec = 0.0;

        public double LMotorPoseRads = 0.0;
        public double LMotorVelRadsPerSec = 0.0;
        public double LMotorAppliedVolts = 0.0;
        public double LMotorSupplyCurrentAmps = 0.0;
        public double LMotorTorqueCurrentAmps = 0.0;
        public double LMotorTempCelsius = 0.0;

        public double RMotorPoseRads = 0.0;
        public double RMotorVelRadsPerSec = 0.0;
        public double RMotorAppliedVolts = 0.0;
        public double RMotorSupplyCurrentAmps = 0.0;
        public double RMotorTorqueCurrentAmps = 0.0;
        public double RMotorTempCelsius = 0.0;
    }
}
