// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MainMechStateMachine;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClosedCmd extends Command {
  /** Creates a new CoralIntake. */

  private MainMechStateMachine m_mainMech;

  public ClosedCmd(MainMechStateMachine mainMech) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_mainMech = mainMech;
    addRequirements(m_mainMech.getArmSubsystem(), m_mainMech.getElevatorSubsystem());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_mainMech.MechStateControl("Closed");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_mainMech.isGoalReached();
  }
}
