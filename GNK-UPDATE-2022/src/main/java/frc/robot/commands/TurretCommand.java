// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.TurretState;

public class TurretCommand extends CommandBase {
  /** Creates a new TurretCommand. */
  private final Shooter boom = Robot.Shooter;
  TurretState State;
  public TurretCommand(TurretState state) {
    this.State = state;
    addRequirements(boom);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    boom.setTurretPower(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(State)
    {
      case Spin:
      boom.setTurretPower(0.7);
      break;
      case ReverseSpin:
      boom.setTurretPower(-0.7);
      break;
      case Stop:
      boom.setTurretPower(0);
      break;
      case Reset:
      boom.setencpos(0);
      break;
      case ninety:
      boom.angleposition(90);
      break;
      case fortyfive:
      boom.angleposition(45);
      break;
      case zero:
      boom.angleposition(0);
      break;
      case negfortyfive:
      boom.angleposition(-45);
      break;
      case negninety:
      boom.angleposition(-90);
      break;
    }
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
