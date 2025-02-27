// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.swerve;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Visualizer {
  public int height, width;

  private final Mechanism2d canvas;

  private final MechanismLigament2d BL, FL, FR, BR;
  private final MechanismLigament2d BLDesired, FLDesired, FRDesired, BRDesired;

  public Visualizer(int height, int width) {
    this.height = height;
    this.width = width;

    canvas = new Mechanism2d(width, height);
    canvas.setBackgroundColor(new Color8Bit(Color.kGray));

    var root = canvas.getRoot("BL", width / 4, height / 4);
    BL = root.append(new MechanismLigament2d("BL-Vector", 30, 90));
    BLDesired =
        root.append(new MechanismLigament2d("BL-DVector", 30, 90, 5, new Color8Bit(Color.kRed)));

    root = canvas.getRoot("FL", width / 4, (3 * height) / 4);
    FL = root.append(new MechanismLigament2d("FL-Vector", 30, 90));
    FLDesired =
        root.append(new MechanismLigament2d("FL-DVector", 30, 90, 5, new Color8Bit(Color.kRed)));

    root = canvas.getRoot("FR", (3 * width) / 4, (3 * height) / 4);
    FR = root.append(new MechanismLigament2d("FR-Vector", 30, 90));
    FRDesired =
        root.append(new MechanismLigament2d("FR-DVector", 30, 90, 5, new Color8Bit(Color.kRed)));

    root = canvas.getRoot("BR", (3 * width) / 4, height / 4);
    BR = root.append(new MechanismLigament2d("BR-Vector", 30, 90));
    BRDesired =
        root.append(new MechanismLigament2d("BR-DVector", 30, 90, 5, new Color8Bit(Color.kRed)));

    SmartDashboard.putData("SwerveVisualizer", canvas);
  }

  public void updateDesired(SwerveModuleState[] states, double scale) {
    if (states.length != 4) {
      System.out.println("Error: visualizer only supports 4 states");
      return;
    }
    BLDesired.setLength(states[0].speedMetersPerSecond * scale);
    BLDesired.setAngle(states[0].angle);

    FLDesired.setLength(states[1].speedMetersPerSecond * scale);
    FLDesired.setAngle(states[1].angle);

    FRDesired.setLength(states[2].speedMetersPerSecond * scale);
    FRDesired.setAngle(states[2].angle);

    BRDesired.setLength(states[3].speedMetersPerSecond * scale);
    BRDesired.setAngle(states[3].angle);
  }

  public void updateReal(SwerveModuleState[] states, double scale) {
    if (states.length != 4) {
      System.out.println("Error: visualizer only supports 4 states");
      return;
    }
    BL.setLength(states[0].speedMetersPerSecond * scale);
    BL.setAngle(states[0].angle);

    FL.setLength(states[1].speedMetersPerSecond * scale);
    FL.setAngle(states[1].angle);

    FR.setLength(states[2].speedMetersPerSecond * scale);
    FR.setAngle(states[2].angle);

    BR.setLength(states[3].speedMetersPerSecond * scale);
    BR.setAngle(states[3].angle);
  }
}
