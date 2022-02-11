// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
//import jdk.vm.ci.meta.Constant;

public class M_LOOP_DRIVE_FRC extends CommandBase {
  private final Drive drive = new Drive();
  private Timer time = new Timer();
  double radius = 0;
  double distance = 0;
  double e = 0;
  double speed = 0;
  double timeout = 0;
  double start = 0;
  double full_circle = 0;
  int direction = 0;
  int cycle = 0;
  int count = 0;
  public double innerside = 0;
  /** Creates a new M_ENCODER_DRIVE_FRC. */
  public M_LOOP_DRIVE_FRC(double inner_circle_radius, double speed, int direction, double timeout, double full_circle) {
    this.radius = inner_circle_radius;
    this.speed = speed;
    this.timeout = timeout;
    this.direction = direction;
    this.full_circle = full_circle;
    e = 18;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.reset();
    time.start();
    if(direction == 0 || direction == 3)
    {
      drive.startpoistion = drive.CountstoInch(2);
      innerside = drive.CountstoInch(3);
    }
    else
    {
      drive.startpoistion = drive.CountstoInch(0);
      innerside = drive.CountstoInch(1);
    }

    distance = radius*3.1415*2;

    if(direction == 2 || direction == 3)
    {
      distance = distance*-1;
    }


  }
  

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    while(time.get() < timeout && Math.abs((distance)*full_circle - innerside) > 0.2)
    {
      drive.LoopEncoderDrive(speed, radius, direction, full_circle, e, -13);

      if(direction == 0 || direction == 3)
    {
      innerside = drive.CountstoInch(3);
    }
    else
    {
      //drive.startpoistion = Constants.rightEncoder.getPosition();
      innerside = drive.CountstoInch(1);
    }

    if(Math.abs(distance*full_circle - innerside) < 2)
    {
      count = count + 1;
      return;
    }
    }
    return;
    
  }

  // Called once the command ends or is interrupted.
  @Override

  public void end(boolean interrupted) {
    drive.robotDrive.tankDrive(0, 0);
    
    if(direction == 0 || direction == 3)
    {
      drive.startpoistion = drive.CountstoInch(2);
      //innerside = drive.CountstoInch(1);
    }
    else
    {
      drive.startpoistion = drive.CountstoInch(0);
      //innerside = drive.CountstoInch(3);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(count < 1)
    {
    return false;
    }
    else
    {
      return true;
    }
  }
}
