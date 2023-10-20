// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsytem extends SubsystemBase {

  private CANSparkMax m_shooterMotor = new CANSparkMax(1, MotorType.kBrushless);
  private SparkMaxPIDController m_shooterController;
  private RelativeEncoder m_shooterEncoder;
  private double m_targetShooterPosition;


  /** Creates a new ArmSubsytem. */
  public ArmSubsytem() {
    m_shooterMotor.restoreFactoryDefaults();
    m_shooterMotor.setIdleMode(IdleMode.kBrake);
    m_shooterEncoder = m_shooterMotor.getEncoder();
    m_shooterController = m_shooterMotor.getPIDController();
    m_shooterEncoder.setPosition(0);
  }

  public void proceedToShooterPosition(){
    double currentDegrees = m_shooterEncoder.getPosition();
    double cosineScalar = Math.cos(Math.toRadians(currentDegrees));
    m_shooterController.setReference(
      m_targetShooterPosition, ControlType.kPosition, 1,
      Constants.SHOOTER_MAX_VOLTAGE_FF * cosineScalar, ArbFFUnits.kVoltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
