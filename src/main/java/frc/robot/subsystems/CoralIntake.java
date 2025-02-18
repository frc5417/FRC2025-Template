// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
  /** Creates a new AlgaeIntake. */
    private final SparkMax coralMotor2;
    private final SparkMax coralMotor3;
    private final SparkMaxConfig coral2Config;

    //private DigitalInput coralIntakeSwitch = new DigitalInput(Constants.ManipulatorConstants.coralIntakeLimitValue);

  public CoralIntake() {
    coralMotor2 = new SparkMax(Constants.ManipulatorConstants.coralMotor2ID, MotorType.kBrushless);
    coralMotor3 = new SparkMax(Constants.ManipulatorConstants.coralMotor3ID, MotorType.kBrushless);
    coral2Config = new SparkMaxConfig();

    coral2Config.inverted(Constants.ManipulatorConstants.coralMotor2Inversion);
  }

  public void setCoralPower(double power) {
    coralMotor2.set(power);
    coralMotor3.set(-power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}