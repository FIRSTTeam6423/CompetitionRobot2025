// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import wmironpatriots.subsystems.elevator.Elevator;
import wmironpatriots.subsystems.tail.Tail;

public class Visualizer {
  private final Elevator elevator;
  private final Tail tail;

  private final Mechanism2d canvas = new Mechanism2d(50, 70);

  private final MechanismRoot2d stageZeroRoot = canvas.getRoot("StageZeroRoot", 25, 0);
  private final MechanismRoot2d stageOneRoot = canvas.getRoot("StageOneRoot", 26, 0);
  private final MechanismRoot2d stageTwoRoot = canvas.getRoot("StageTwoRoot", 27, 0);
  private final MechanismLigament2d stageZero =
      stageZeroRoot.append(
          new MechanismLigament2d("StageZero", 32, 90, 5.0, new Color8Bit(Color.kRed)));

  private final MechanismLigament2d stageOne =
      stageOneRoot.append(
          new MechanismLigament2d("StageOne", 33, 90, 5.0, new Color8Bit(Color.kOrange)));

  private final MechanismLigament2d stageTwo =
      stageTwoRoot.append(
          new MechanismLigament2d("StageTwo", 6.973556/2, 90, 5.0, new Color8Bit(Color.kGreen)));

  private final MechanismLigament2d tailPivot =
      stageTwo.append(new MechanismLigament2d("Tail", Tail.LENGTH_INCHES, 0));

  public Visualizer(Elevator elevator, Tail tail) {
    this.elevator = elevator;
    this.tail = tail;
    
    stageTwo.append(new MechanismLigament2d("w", 6.973556/2, 0, 5.0, new Color8Bit(Color.kGreen)));

    SmartDashboard.putData("Visualizer", canvas);
  }

  /** Update visualizer */
  public void periodic() {
    tailPivot.setAngle((tail.getPose() * (180 / Math.PI)) - 90);
    double converted =
        (elevator.getPose() * 2 * Math.PI * Elevator.SPOOL_RADIUS_INCHES) / Elevator.REDUCTION;
    stageOneRoot.setPosition(26, converted);
    stageTwoRoot.setPosition(27, converted * 2);
  }
}
