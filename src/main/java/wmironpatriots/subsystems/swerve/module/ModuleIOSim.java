// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.swerve.module;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ModuleIOSim extends Module {
  private static final DCMotor pivotMotor = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor driveMotor = DCMotor.getKrakenX60Foc(1);

  private final DCMotorSim pivotSim, driveSim;

  public ModuleIOSim() {}

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'periodic'");
  }

  @Override
  protected void setPivotCurrent(double amps) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setPivotCurrent'");
  }

  @Override
  protected void setDriveCurrent(double amps, boolean focEnabled) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setDriveCurrent'");
  }

  @Override
  protected void setPivotPose(double poseRevs) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setPivotPose'");
  }

  @Override
  protected void setDriveVel(double velMPS) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setDriveVel'");
  }

  @Override
  public void stopMotors() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'stopMotors'");
  }

  @Override
  protected void enableCoastMode(boolean enabled) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'enableCoastMode'");
  }
}
