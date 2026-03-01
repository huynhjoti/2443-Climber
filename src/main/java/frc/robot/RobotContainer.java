// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.climber.Climber;

import frc.robot.commands.VerticalCmd;

public class RobotContainer {

  Climber climber;
  CommandXboxController xbox;

  public RobotContainer() {
    climber = new Climber(14, 15, 1, 2, 0, 0);
    xbox = new CommandXboxController(0);
    configureBindings();
  }

  private void configureBindings() {
    //Elevator Manual Buttons
    // xbox.a().whileTrue(climber.up());
    // xbox.a().whileFalse(climber.stop());
    // xbox.b().whileTrue(climber.down());
    // xbox.b().whileFalse(climber.stop());

    //Pivot Manual Buttons 
    // xbox.a().whileTrue(climber.clockwise());
    // xbox.a().whileFalse(climber.stopPivot());
    // xbox.b().whileTrue(climber.counterClockWise());
    // xbox.b().whileFalse(climber.stopPivot());
    // xbox.x().onTrue(new VerticalCmd(climber, 25));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
