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
public class IMU_DRIVE extends SequentialCommandGroup {
  /** Creates a new IMU_DRIVE. */
  public IMU_DRIVE() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new M_IMU_DRIVE(120, 20, .3, 30)
    );
  }
}
