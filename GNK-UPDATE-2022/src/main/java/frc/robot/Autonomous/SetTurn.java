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
public class SetTurn extends SequentialCommandGroup {
  /** Creates a new Turn. */
  public SetTurn() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
         
    new M_IMU_DRIVE(118, 0, .6, 7),
    new M_LOOP_DRIVE_FRC(20, .45, 1, 5, 1.01),
    new M_IMU_DRIVE(90, 0, .6, 7),
    new M_LOOP_DRIVE_FRC(20, .45, 0, 5, .93),
    new M_IMU_DRIVE(112, 44, .6, 7),
    new M_LOOP_DRIVE_FRC(20, .45, 0, 5, .68),
    new M_IMU_DRIVE(260, 180, .7, 7)
    /* 

    new M_ENCODER_DRIVE_FRC(110, .5, 3),
    new M_IMU_TURN_FRC(.5, 0, 2),
    new M_ENCODER_DRIVE_FRC(130, .5, 3),
    new M_IMU_TURN_FRC(.5, 270, 2),
    new M_ENCODER_DRIVE_FRC(75, .5, 3),
    new M_IMU_TURN_FRC(.5, 180, 2),
    new M_ENCODER_DRIVE_FRC(300, .6, 7)
    */
    //new M_ENCODER_DRIVE_FRC(-20, .7, 4)
   // new M_IMU_TURN_FRC(.3, 20, 2),
   // new M_ENCODER_DRIVE_FRC(100, .5, 5
   );
  }
}
