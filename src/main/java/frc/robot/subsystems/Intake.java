// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//se ponen las librerias


public class Intake extends SubsystemBase {
  private final CANSparkMax MotorIntake = new CANSparkMax(8, MotorType.kBrushless);
  // Se declaran motores
  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void SetMotors(double speed){
    MotorIntake.set(speed);
    //Función que le da la potencia a los motores 
  }
}
