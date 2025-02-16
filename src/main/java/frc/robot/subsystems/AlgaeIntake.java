// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
  /** Creates a new AlgaeIntake. */
    public final SparkMax algaeMotor0;
    public final SparkMax algaeMotor1;

  public AlgaeIntake() {
    algaeMotor0 = new SparkMax(Constants.ManipulatorConstants.algaeMotor0ID, MotorType.kBrushless);
    algaeMotor1 = new SparkMax(Constants.ManipulatorConstants.algaeMotor1ID, MotorType.kBrushless);
  }

  public void setAlgaePower(double power) {
    algaeMotor0.set(power);
    algaeMotor1.set(-power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
