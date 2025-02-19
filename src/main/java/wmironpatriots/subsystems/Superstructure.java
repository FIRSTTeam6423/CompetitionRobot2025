// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import wmironpatriots.subsystems.elevator.Elevator;

/** Possibly one of the most cursed superstructures fr */
public class Superstructure {
    public static enum StructState {
        IDLE,
        INTAKE_CHUTE,
        INTAKE_GROUND,
        L1_SETUP,
        L2_SETUP,
        L3_SETUP,
        L4_SETUP,
        REEF_SCORE,
        CORAL_OUTTAKE,
        ALGAE_HIGHER,
        ALGAE_LOWER,
        PROCESSOR_SETUP,
        PROCESSOR_SCORE,
        ALGAE_OUTTAKE
    }

    private final Map<StructState, Trigger> m_triggerMap = new HashMap<StructState, Trigger>();
    private StructState m_currentState = StructState.IDLE;

    public Superstructure(Elevator elevator, Map<StructState, Trigger> triggerMap) {
        for (var state : StructState.values()) {
            Trigger trigger = triggerMap.get(state);
            if (trigger == null) {
                m_triggerMap.put(state, new Trigger(() -> this.m_currentState == state));
            } else {
                m_triggerMap.put(state, trigger);
            }
        } // This is really dumb yippe

        /** INTAKING GAMEPIECES */
        
        /** CORAL SCORING SETUP */
        triggerMap
            .get(StructState.L1_SETUP)
            .onTrue(new WaitCommand(0.0)) // Tail logic
            .toggleOnFalse(setCurrentStateCommand(StructState.IDLE)); 

        triggerMap
            .get(StructState.L2_SETUP)
            .whileTrue(elevator.runTargetPoseCommand(Elevator.kL2Pose))
            .and(elevator::inSetpointRange)
            .onTrue(new WaitCommand(0.0)) // Tail logic
            .toggleOnFalse(setCurrentStateCommand(StructState.IDLE)); 

        triggerMap
            .get(StructState.L3_SETUP)
            .whileTrue(elevator.runTargetPoseCommand(Elevator.kL3Pose))
            .and(elevator::inSetpointRange)
            .onTrue(new WaitCommand(0.0)) // Tail logic
            .toggleOnFalse(setCurrentStateCommand(StructState.IDLE)); 

        triggerMap
            .get(StructState.L4_SETUP)
            .whileTrue(elevator.runTargetPoseCommand(Elevator.kL4Pose))
            .and(elevator::inSetpointRange)
            .onTrue(new WaitCommand(0.0)) // Tail logic
            .toggleOnFalse(setCurrentStateCommand(StructState.IDLE)); 
    }

    /** Set current state */
    public Command setCurrentStateCommand(StructState desiredState) {
        return Commands.run(() -> {
            m_currentState = desiredState;
        });
    }
}
