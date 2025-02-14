package org.frc6423.frc2025.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ArmIO {
    @AutoLog
    public class ArmIOInputs {
        public boolean pivotMotorEnabled = false;
        public boolean rollerMotorEnabled = false;
    }   
    
    /** Run pivot motor voltage */
    public void runVolts(double voltage); 

    /** Run drive motor voltage */
    public void runRollerVolts(double voltage);

    /** Run arm to specific angle */
    public void runTargetPose(Rotation2d targetAngle);

    /** Set target roller velocity in rads per second */
    public void runTargetRollerVeloctiy(double radsPerSec);
}
