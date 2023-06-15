// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.commands.Autonomo;
import frc.robot.commands.Balancecmmd;
import frc.robot.commands.Chasiscmmd;
import frc.robot.commands.Elevatorcmmd;
import frc.robot.commands.Giro;
import frc.robot.commands.Intakecmmd;
import frc.robot.commands.Visioncmmd;
import frc.robot.subsystems.Chasis;
import frc.robot.subsystems.ElevatorChasis;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command; 
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
//Aqui van todas las librerias que se necesitan y se importan los subsistemas 
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  private final XboxController JoyDrive = new XboxController(0);
  private final XboxController SadDrive = new XboxController(1);

  //private final ChasisDriveSubsystem ChasisSub = new ChasisDriveSubsystem();
  private final ElevatorChasis Elevator = new ElevatorChasis();
  private final Chasis Chasiscontrol= new Chasis();
  private final Intake IntakeShooter= new Intake();
  
  //se realizan los builders de controles Joy drive = control de chasis y Saddrive = control para subsistemas 
  // Se hace el builder para los subsistemas y sus comandos 
  public RobotContainer() {
    //ChasisSub.setDefaultCommand(new DriveJoystickCmmd(ChasisSub, () -> JoyDrive.getRawAxis(3), () -> -JoyDrive.getRawAxis(2) ,() -> JoyDrive.getRawAxis(0),() -> JoyDrive.getAButton()));
    Chasiscontrol.setDefaultCommand(new Chasiscmmd(Chasiscontrol, () -> JoyDrive.getRawAxis(3), () -> JoyDrive.getRawAxis(0), () -> JoyDrive.getRawAxis(2), ()-> JoyDrive.getAButton()));
    //Elevator.setDefaultCommand(new Elevatorcmmd(Elevator, () -> SadDrive.getRightTriggerAxis(), ()-> SadDrive.getLeftTriggerAxis()));
    //IntakeShooter.setDefaultCommand (new Intakecmmd(IntakeShooter,() -> SadDrive.getRawAxis(1)));
    configureBindings();
    //Se declaran los subsistemas y cuando es que se usan junto con los valores que tendrán los supliers 
  }

  private void configureBindings() {
    //new JoystickButton(JoyDrive, 1).whileTrue(new FowardChasisPidCmmd(ChasisSub, 100));
    new JoystickButton(SadDrive, 1).whileTrue(new Visioncmmd(Chasiscontrol));
    new JoystickButton(SadDrive, 2).whileTrue(new Giro(Chasiscontrol));

  }


  public Command getAutonomousCommand() {
    //return new FowardChasisPidCmmd(ChasisSub, 100);
    //return new Autonomo(Chasiscontrol, 20);
    return new Balancecmmd(Chasiscontrol, 30);
    //Se cambia el setpoint a 120 ya que sea la prueba en cancha 
    
  }
}
