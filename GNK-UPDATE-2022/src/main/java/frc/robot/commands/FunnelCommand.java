// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.Funnel.FunnelState;

public class FunnelCommand extends CommandBase {
  /** Creates a new FunnelCommand. */
  private final Funnel dumpydump = Robot.funnel;
  FunnelState State;
  double howmuchpowerdoyouwant = 0;
    public FunnelCommand(FunnelState E, double miss_keisha) {
    howmuchpowerdoyouwant = miss_keisha; 
    this.State = E;
    addRequirements(dumpydump);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(State)
    {
      case Collect:
      
      if(dumpydump.getRamp() < 3 && dumpydump.getStage2() < 1.5 && dumpydump.getStage1() < 1.5)
      {
        dumpydump.setRamp(howmuchpowerdoyouwant);
        dumpydump.setRoller1(howmuchpowerdoyouwant);
        dumpydump.setRoller2(howmuchpowerdoyouwant);
        dumpydump.setIntake(howmuchpowerdoyouwant);
        //return;
      }
      else if(dumpydump.getRamp() > 3 && dumpydump.getStage2() < 1.5 && dumpydump.getStage1() < 1.5)
      {
        dumpydump.setRamp(0);
        dumpydump.setRoller1(howmuchpowerdoyouwant);
        dumpydump.setRoller2(howmuchpowerdoyouwant);
        dumpydump.setIntake(howmuchpowerdoyouwant);
        //return;
      }
      else if(dumpydump.getRamp() > 3 && dumpydump.getStage2() > 1.5 && dumpydump.getStage1() < 1.5)
      {
        dumpydump.setRamp(0);
        dumpydump.setRoller1(0);
        dumpydump.setRoller2(howmuchpowerdoyouwant);
        dumpydump.setIntake(howmuchpowerdoyouwant);
        //return;
      }
      else if(dumpydump.getRamp() > 3 && dumpydump.getStage2() > 1.5 && dumpydump.getStage1() > 1.5)
      {
        dumpydump.setRamp(0);
        dumpydump.setRoller1(0);
        dumpydump.setRoller2(0);
        dumpydump.setIntake(howmuchpowerdoyouwant);
        //return;
      }
      break;
      case Remove:
        if(dumpydump.getRamp() > 3)
        {
        dumpydump.setRamp(howmuchpowerdoyouwant);
        dumpydump.setRoller1(0);
        dumpydump.setRoller2(0);
        dumpydump.setIntake(0);
        }
        else
        {
        dumpydump.setRamp(howmuchpowerdoyouwant);
        dumpydump.setRoller1(howmuchpowerdoyouwant);
        dumpydump.setRoller2(howmuchpowerdoyouwant);
        dumpydump.setIntake(howmuchpowerdoyouwant);
        }
      break;
      case Stop:
       dumpydump.setRamp(0);
       dumpydump.setRoller1(0);
       dumpydump.setRoller2(0);
       dumpydump.setIntake(0)
       ;
       break;
    }
    return;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
