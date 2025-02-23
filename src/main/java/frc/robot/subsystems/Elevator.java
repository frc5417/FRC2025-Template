// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//44eeimport edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final SparkFlex elevatorParent;
    private final SparkFlex elevatorChild;
    public final RelativeEncoder elevatorParentEncoder;
    public final RelativeEncoder elevatorChildEncoder;

    public Elevator() {
        elevatorParent = new SparkFlex(Constants.Elevator.elevatorParentId, MotorType.kBrushless);
        elevatorChild = new SparkFlex(Constants.Elevator.elevatorChildId, MotorType.kBrushless);

        motorConfig();

        elevatorParentEncoder = elevatorParent.getEncoder();
        elevatorChildEncoder = elevatorChild.getEncoder();
    }

    public void setElevatorPower(double power) {
        elevatorParent.set(power);
        elevatorChild.set(-power);
    }
    
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      //super.periodic();
      SmartDashboard.putNumber("Elevator Parent (54) Encoder", elevatorParentEncoder.getPosition());
      SmartDashboard.putNumber("Elevator Child (55) Encoder", elevatorChildEncoder.getPosition());
    }

    /**
     * Configures the elevator motors.
     */
    private void motorConfig() {
        SparkFlexConfig parentConfig = new SparkFlexConfig();
        SparkFlexConfig childConfig = new SparkFlexConfig();

        parentConfig.idleMode(IdleMode.kBrake);
        childConfig.apply(parentConfig);
        childConfig.inverted(Constants.Elevator.elevatorChildInversion);
    }
}
