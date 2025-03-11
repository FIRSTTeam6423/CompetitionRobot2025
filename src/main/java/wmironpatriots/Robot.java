// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import wmironpatriots.subsystems.Superstructure;
import wmironpatriots.subsystems.chute.Chute;
import wmironpatriots.subsystems.chute.ChuteIOComp;
import wmironpatriots.subsystems.elevator.Elevator;
import wmironpatriots.subsystems.elevator.ElevatorIOComp;
import wmironpatriots.subsystems.tail.Tail;
import wmironpatriots.subsystems.tail.TailIOComp;

public class Robot extends TimedRobot {
  private final Superstructure superstructure;
  private final Elevator elevator;
  private final Tail tail;
  private final Chute chute;

  private final CommandXboxController operator;

  public Robot() {
    operator = new CommandXboxController(1);

    elevator = new ElevatorIOComp();
    tail = new TailIOComp();
    chute = new ChuteIOComp();

    superstructure = new Superstructure(elevator, tail, chute);

    elevator.setDefaultCommand(superstructure.defaultElevatorCmmd());
    tail.setDefaultCommand(superstructure.defaultTailCmmd());

    operator.x().whileTrue(elevator.runPoseCmmd(Elevator.POSE_L3));
    operator.a().whileTrue(tail.runPoseCmmd(Tail.POSE_MAX));
    operator.y().whileTrue(superstructure.intakeCoral());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}
}
