// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ManualClimb;

import edu.wpi.first.wpilibj.XboxController;

import frc.robot.climber.Climber;

public class RobotContainer {

  Climber climber;
  XboxController xbox;

  public RobotContainer() {
    climber = new Climber(0, 0, 0, 0);
    xbox = new XboxController(0);
    configureBindings();
  }

  private void configureBindings() {
    climber.setDefaultCommand(new ManualClimb(climber, () -> xbox.getLeftY()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
