// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import wmironpatriots.subsystems.superstructure.arm.Arm;
import wmironpatriots.subsystems.superstructure.climber.Climber;
import wmironpatriots.subsystems.superstructure.elevator.Elevator;

public class Superstructure {
  private final Elevator elevator;
  private final Arm arm;
  private final Climber climber;

  public Superstructure() {
    elevator = Elevator.createElevator();
    arm = new Arm();
    climber = new Climber();

    elevator.setDefaultCommand(elevatorDefaultCmd());
    arm.setDefaultCommand(armDefaultCmd());
    climber.setDefaultCommand(climberDefaultCmd());
  }

  // * SUBSYSTEM DEFAULT CMDS
  private Command elevatorDefaultCmd() {
    return Commands.sequence(
        elevator.runCurrentZeroingCmd().onlyIf(elevator::isInitalized),
        elevator
            .runPoseCmd(1)
            .until(elevator::nearSetpoint)
            .finallyDo(() -> elevator.stopMotors()));
  }

  private Command armDefaultCmd() {
    return Commands.run(() -> {}, arm);
  }

  private Command climberDefaultCmd() {
    return Commands.run(() -> {}, climber);
  }
}
