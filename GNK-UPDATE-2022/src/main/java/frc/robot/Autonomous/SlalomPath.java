// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.M_IMU_DRIVE;
import frc.robot.commands.M_LOOP_DRIVE_FRC;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SlalomPath extends SequentialCommandGroup {
  /** Creates a new SlalomPath. */
  public SlalomPath() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new M_IMU_DRIVE(35, 0, .6, 3),
      new M_LOOP_DRIVE_FRC(20, .3, 0, 5, .25),
      new M_LOOP_DRIVE_FRC(20, .3, 1, 5, .3),
      new M_IMU_DRIVE(118, 0, .6, 5),
      new M_LOOP_DRIVE_FRC(20, .3, 1, 5, .29),
      new M_LOOP_DRIVE_FRC(20, .3, 0, 10, 1.08),
      new M_LOOP_DRIVE_FRC(20, .3, 1, 5, .27),
      new M_IMU_DRIVE(119, 180, .6, 5),
      new M_LOOP_DRIVE_FRC(20, .3, 1, 5, .26),
      new M_LOOP_DRIVE_FRC(20, .3, 0, 5, .3),
      new M_IMU_DRIVE(20, 180, .5, 3)
    );
  }
}
