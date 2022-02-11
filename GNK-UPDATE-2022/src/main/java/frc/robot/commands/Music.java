// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Shooter;

public class Music extends CommandBase {
  /** Creates a new Music. */
  private final Shooter sing = Robot.Shooter;
  public Music() {
    addRequirements(sing);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  sing.setTurretPower(0);
  sing.setPower(0);
  if(sing.current_music == sing.max_music)
  {
   sing.current_music = 0;
  }
  sing.current_music = sing.current_music + 1;

  sing.loadmusic(sing.current_music);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sing.playmusic();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
