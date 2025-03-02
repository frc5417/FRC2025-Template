// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunElevator extends Command {
  private double power;
  private Elevator m_elevator;
  private boolean terminate = false;
  /** Creates a new RunElevator. */
  public RunElevator(Elevator elevator) {
    m_elevator = elevator;
    power = 0;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    power = RobotContainer.getManipulatorRightJoyY();
    m_elevator.setElevatorPower(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.setElevatorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return terminate;
  }
}
