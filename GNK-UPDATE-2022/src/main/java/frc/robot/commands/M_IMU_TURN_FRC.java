// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Drive;

public class M_IMU_TURN_FRC extends CommandBase {
  private final Drive drive = Robot.drive;
  double angle;
  double speed;
  int clock;
  int count = 0;
  double error = 0;
  private Timer timey = new Timer();
  /** Creates a new M_IMU_TURN_FRC. */
  public M_IMU_TURN_FRC(double power, double yaw, int time) {
    angle = yaw;
    speed = power;
    clock = time;
    //Constants.imu.zeroYaw();
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timey.reset();
    timey.start();
    //Constants.imu.zeroYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = Math.abs(drive.getError(angle));
    while(timey.get() < clock && count < 35)
    {
    drive.ImuTurn(speed, angle);
    error = Math.abs(drive.getError(angle));

    if(error < 1)
    {
      count = count + 1;
    } 

    SmartDashboard.putNumber("count", count);

    }
    return;
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.robotDrive.tankDrive(0, 0);
    //timey.delay(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(count < 1 && timey.get() < clock) 
    {
      return false;
    }
    else if(count > 1)
    {
      return true;
    }
    else 
    {
      return true;
    }
  }
}
