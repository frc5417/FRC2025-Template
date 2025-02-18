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
    private final SparkMax algaeMotor0;
    private final SparkMax algaeMotor1;
    private final SparkMaxConfig algae0Config;
    //private final SparkMaxConfig algae1Config;
    private DigitalInput algaeIntakeSwitch = new DigitalInput(Constants.ManipulatorConstants.algaeIntakeLimitValue);

  public AlgaeIntake() {
    algaeMotor0 = new SparkMax(Constants.ManipulatorConstants.algaeMotor0ID, MotorType.kBrushless);
    algaeMotor1 = new SparkMax(Constants.ManipulatorConstants.algaeMotor1ID, MotorType.kBrushless);
    algae0Config = new SparkMaxConfig();

    algae0Config.inverted(Constants.ManipulatorConstants.algaeMotor0Inversion);
  }

  public void setAlgaePower(double power) {
    if (algaeIntakeSwitch.get()) {
      algaeMotor0.set(0);
      algaeMotor1.set(0);
    }
    else {
      algaeMotor0.set(power);
      algaeMotor1.set(-power);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
}