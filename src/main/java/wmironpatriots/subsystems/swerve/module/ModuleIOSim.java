// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.swerve.module;

import com.ctre.phoenix6.controls.ControlRequest;
import wmironpatriots.util.swerveUtil.ModuleConfig;

public class ModuleIOSim extends Module {

  public ModuleIOSim(ModuleConfig config) {
    super(config);
  }

  @Override
  protected void runPivotControl(ControlRequest request) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'runPivotControl'");
  }

  @Override
  protected void runDriveControl(ControlRequest request) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'runDriveControl'");
  }

  @Override
  protected void stopMotors() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'stopMotors'");
  }

  @Override
  protected void motorCoasting(boolean enabled) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'motorCoasting'");
  }
}
