// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class M_IMU_DRIVE extends CommandBase {
  private final Drive drive = new Drive();
  private Timer time = new Timer();
  double distance = 0;
  double speed = 0;
  double timeout = 0;
  double angle = 0;
  double start = 0;
  int cycle = 0;
  public int county = 0;
  /** Creates a new M_ENCODER_DRIVE_FRC. */
  public M_IMU_DRIVE(double distance, double angle, double speed, double timeout) {
    this.distance = distance;
    this.speed = speed;
    this.timeout = timeout;
    this.angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.reset();
    time.start();
    drive.startpoistion = drive.CountstoInch(4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    while(time.get() < timeout && Math.abs(distance - drive.CountstoInch(5)) > 1 && county < 1)
    {
      drive.ImuDrive(distance, angle, speed);
      SmartDashboard.putNumber("County", county);

      if(Math.abs(distance - drive.CountstoInch(5)) < 1)
      {
        county = county + 1;

      }
      else if(county == 1)
      {
        return;
      }
    }
    return;
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.robotDrive.tankDrive(0, 0);
    drive.startpoistion = drive.CountstoInch(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(county >= 1)
    {
    return true;
    }
    else
    {
      return false;
    }
  }
}
