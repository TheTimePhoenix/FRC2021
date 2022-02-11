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
public class BouncyPath extends SequentialCommandGroup {
  /** Creates a new BouncyPath. */
  public BouncyPath() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //you hear that. literally no one can understand the reasoning behind naming your variables :)
    addCommands(
      
      new M_IMU_DRIVE(30, 0, .6, 3),
      new M_LOOP_DRIVE_FRC(20, .3, 0, 5, .25),
      new M_IMU_DRIVE(34, 270, .8, 3),
      new M_IMU_DRIVE(-100, 250, .6, 3),
      new M_LOOP_DRIVE_FRC(20, .4, 2, 5, .45),
      new M_IMU_DRIVE(-110, 90, .6, 5),
      new M_IMU_DRIVE(85, 90, .6, 5),
      new M_LOOP_DRIVE_FRC(20, .3, 0, 5, .25),
      new M_IMU_DRIVE(17, 0, .5, 3),
      new M_LOOP_DRIVE_FRC(20, .3, 0, 5, .25),
      new M_IMU_DRIVE(110, 270, .6, 4),
      new M_IMU_DRIVE(-30, 270, .6, 4),
      new M_LOOP_DRIVE_FRC(20, .4, 2, 5, .25)
    );
  }
}
