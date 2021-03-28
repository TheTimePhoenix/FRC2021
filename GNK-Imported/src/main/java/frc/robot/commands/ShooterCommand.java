// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;

public class ShooterCommand extends CommandBase {
  /** Creates a new ShooterCommand. */
  private final Shooter shoot = Robot.Shooter;
  ShooterState state;
  public ShooterCommand(ShooterState State) {
    this.state = State;
    addRequirements(shoot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(state){
      case Shoot:
      shoot.setPower(0.8);
      break;
      case Regeritate:
      shoot.setPower(-0.8);
      break;
      case Stop:
      shoot.setPower(0);
      break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoot.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
