// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.AlgaeIntake;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunAlgae extends Command {
  private final double power;
  private static AlgaeIntake algaeIntake;
  private boolean terminate = false;

  /** Creates a new RunAlgae. */
  public RunAlgae(AlgaeIntake intake, double power) {
    algaeIntake = intake;
    this.power = power;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    algaeIntake.setAlgaePower(power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    terminate = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeIntake.setAlgaePower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return terminate;
  }
}
