// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
  /** Creates a new AlgaeIntake. */
    private final SparkMax coralWrist;
    private final SparkMax coralWheel;

    //private DigitalInput coralIntakeSwitch = new DigitalInput(Constants.ManipulatorConstants.coralIntakeLimitValue);

  public CoralIntake() {
    coralWrist = new SparkMax(Constants.ManipulatorConstants.coralWrist, MotorType.kBrushless);
    coralWheel = new SparkMax(Constants.ManipulatorConstants.coralWheel, MotorType.kBrushless);

    configMotors();
  }

  public void setCoralWristPower(double power) {
    coralWrist.set(power);
  }

  public void setCoralWheelPower(double power) {
    coralWheel.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void configMotors() {
    SparkMaxConfig wristConfig = new SparkMaxConfig();
    SparkMaxConfig wheelConfig = new SparkMaxConfig();

    wristConfig.idleMode(IdleMode.kBrake);
    wheelConfig.idleMode(IdleMode.kBrake);

    wristConfig.smartCurrentLimit(Constants.MotorConstants.kNeoCL);
    wheelConfig.smartCurrentLimit(Constants.MotorConstants.kNeo550CL);

    coralWrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    coralWheel.configure(wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}