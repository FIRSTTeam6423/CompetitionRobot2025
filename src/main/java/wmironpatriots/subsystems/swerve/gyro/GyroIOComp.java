// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.swerve.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

public class GyroIOComp extends Gyro {
  private final Pigeon2 pigeon;
  private final BaseStatusSignal sigYaw, sigPitch, sigRoll; // , sigYawVel, sigPitchVel, sigRollVel;

  public GyroIOComp() {
    super();

    pigeon = new Pigeon2(GYRO_ID);

    sigYaw = pigeon.getYaw();
    sigPitch = pigeon.getPitch();
    sigRoll = pigeon.getRoll();
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(sigYaw, sigPitch, sigRoll);

    double degToRads = (Math.PI * 180);
    yawRads = sigYaw.getValueAsDouble() * degToRads;
    pitchRads = sigPitch.getValueAsDouble() * degToRads;
    rollRads = sigRoll.getValueAsDouble() * degToRads;
  }
}
