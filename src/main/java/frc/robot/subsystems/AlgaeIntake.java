// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
  /** Creates a new AlgaeIntake. */
    private final SparkMax algaeParent;
    private final SparkMax algaeChild;
    private final SparkMaxConfig algaeParentConfig;
    //private final SparkMaxConfig algae1Config;
    private DigitalInput algaeIntakeSwitch = new DigitalInput(Constants.ManipulatorConstants.algaeLimitValue);

  public AlgaeIntake() {
    algaeParent = new SparkMax(Constants.ManipulatorConstants.algaeParentId, MotorType.kBrushless);
    algaeChild = new SparkMax(Constants.ManipulatorConstants.algaeChildId, MotorType.kBrushless);
    algaeParentConfig = new SparkMaxConfig();

    algaeParentConfig.inverted(Constants.ManipulatorConstants.algaeChildInversion);
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
  
}