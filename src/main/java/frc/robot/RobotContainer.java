// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.commands.DriveJoystickCmmd;
import frc.robot.commands.FowardChasisPidCmmd;
import frc.robot.subsystems.ChasisDriveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {
  private final XboxController JoyDrive = new XboxController(0);
  private final ChasisDriveSubsystem ChasisSub = new ChasisDriveSubsystem();
  
  public RobotContainer() {
    ChasisSub.setDefaultCommand(new DriveJoystickCmmd(ChasisSub, () -> JoyDrive.getRawAxis(3), () -> -JoyDrive.getRawAxis(2) ,() -> JoyDrive.getRawAxis(0),() -> JoyDrive.getAButton()));
    configureBindings();
  }

  private void configureBindings() {
    //new JoystickButton(JoyDrive, 1).whileTrue(new FowardChasisPidCmmd(ChasisSub, 100));
  }

  
  public Command getAutonomousCommand() {
    return new FowardChasisPidCmmd(ChasisSub, 100);
    
  
  }
}
