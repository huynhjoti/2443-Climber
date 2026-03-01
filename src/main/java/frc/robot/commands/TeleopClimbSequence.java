// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.climber.Climber;

import frc.robot.commands.RatchetPassiveHooks;
import frc.robot.commands.ReachForRung;
import frc.robot.commands.GoToTop;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TeleopClimbSequence extends SequentialCommandGroup {
  /** Creates a new TeleopHang. */
  public Climber climber;
  public TeleopClimbSequence(Climber newClimber) {
    climber = newClimber;
    addRequirements(climber);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RatchetPassiveHooks(climber),
      new ReachForRung(climber),
      new GoToTop(climber),
      new ReachForRung(climber),
      new GoToTop(climber),
      new ReachForRung(climber)
      ); 
  }
}
