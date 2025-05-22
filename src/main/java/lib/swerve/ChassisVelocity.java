package lib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Represents a velocity of a chassis composed of 3 components:
 * <p> - X speed
 * <p> - Y speed
 * <p> - Angular rate
 */
public class ChassisVelocity {
   public double vxMetersPerSec, vyMetersPerSec, omegaRadsPerSec; 

   private ChassisVelocity(double vxMetersPerSec, double vyMetersPerSec, double omegaRadsPerSec) {
      this.vxMetersPerSec = vxMetersPerSec;
      this.vyMetersPerSec = vyMetersPerSec;
      this.omegaRadsPerSec = omegaRadsPerSec;
   }

   /**
    * Create a new chassis velocity from 3 speeds
    * 
    * @param vxMetersPerSec The x speed component
    * @param vyMetersPerSec The y speed component
    * @param omegaRadsPerSec The angular rate
    * @return {@link ChassisVelocity} representing the resulting velocity
    */
   public static ChassisVelocity fromRobotRelativeSpeeds(double vxMetersPerSec, double vyMetersPerSec, double omegaRadsPerSec) {
      return new ChassisVelocity(vxMetersPerSec, vyMetersPerSec, omegaRadsPerSec);
   }

   /**
    * Converts a {@link ChassisSpeeds} to a {@link ChassisVelocity}
    * 
    * @param speeds {@link ChassisSpeed} to convert
    * @return {@link ChassisVelocity} represnting the converted {@link ChassisSpeeds}
    */
   public static ChassisVelocity fromRobotRelativeSpeeds(ChassisSpeeds speeds) {
      return new ChassisVelocity(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
   }

   /**
    * Creates a new chassis velocity from 3 field-relative speeds (where a positive x speed points away from the alliance wall) to robot-relative speeds
    * 
    * @param vxMetersPerSec The x field-relative speed component
    * @param vyMetersPerSec The y field-relative speed component
    * @param omegaRadsPerSec The angular rate
    * @param headingRads The angle between the positive x axis and the positive y axis of the chassis
    * @return {@link ChassisVelocity} representing the resulting velocity
    */
   public static ChassisVelocity fromFieldRelativeSpeeds(
      double vxMetersPerSec, 
      double vyMetersPerSec, 
      double omegaRadsPerSec, 
      double headingRads) {
      return new ChassisVelocity(
         vxMetersPerSec * Math.cos(headingRads) - vyMetersPerSec * Math.sin(headingRads), 
         vxMetersPerSec * Math.sin(headingRads) + vyMetersPerSec * Math.cos(headingRads), 
         omegaRadsPerSec);
   }

   public static ChassisVelocity discretize(ChassisVelocity continousVelocity, double dtSeconds) {
      return ChassisVelocity.fromRobotRelativeSpeeds(
         continousVelocity.vxMetersPerSec * dtSeconds, 
         continousVelocity.vyMetersPerSec * dtSeconds, 
         continousVelocity.omegaRadsPerSec * dtSeconds);
   }

   /**
    * @return {@link ChassisSpeeds} representing the components of this {@link ChassisVelocity}
    */
   public ChassisSpeeds getRobotRelative() {
      return new ChassisSpeeds(vxMetersPerSec, vyMetersPerSec, omegaRadsPerSec);
   }

   /**
    * Creates a {@link ChassisSpeeds} from the field-relative components of this {@link ChassisVelocity}
    * 
    * @param headingRads The angle between the positive x axis and the positive y axis of the chassis
    * @return
    */
   public ChassisSpeeds getFieldRelative(double headingRads) {
      return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelative(), Rotation2d.fromRadians(headingRads));
   }

   /**
    * @return the scaler component of the linear velocity
    */
   public double getLinearSpeed() {
      return Math.sqrt(Math.pow(vxMetersPerSec, 2) + Math.pow(vyMetersPerSec, 2));
   }
}