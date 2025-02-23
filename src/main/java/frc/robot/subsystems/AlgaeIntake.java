// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
  /** Creates a new AlgaeIntake. */
    private final SparkMax algaeParent;
    private final SparkMax algaeChild;
    //private final SparkMaxConfig algae1Config;
    private DigitalInput algaeIntakeSwitch = new DigitalInput(Constants.ManipulatorConstants.algaeLimitValue);

  public AlgaeIntake() {
    algaeParent = new SparkMax(Constants.ManipulatorConstants.algaeParentId, MotorType.kBrushless);
    algaeChild = new SparkMax(Constants.ManipulatorConstants.algaeChildId, MotorType.kBrushless);
    
    configMotors();
  }

  public void setAlgaePower(double power) {
    if (algaeIntakeSwitch.get()) {
      algaeParent.set(0);
      algaeChild.set(0);
    }
    else {
      algaeParent.set(power);
      algaeChild.set(-power);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void configMotors() {
    SparkMaxConfig parentConfig = new SparkMaxConfig();
    SparkMaxConfig childConfig = new SparkMaxConfig();

    parentConfig.idleMode(IdleMode.kBrake);
    parentConfig.smartCurrentLimit(25);
    algaeParent.configure(parentConfig, ResetMode.kResetSafeParameters, null);

    childConfig.apply(parentConfig);
    childConfig.inverted(true);
    algaeChild.configure(childConfig, ResetMode.kResetSafeParameters, null);
  }
}