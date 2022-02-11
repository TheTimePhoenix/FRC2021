// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public enum ShooterState {Shoot, Regeritate, Stop};
  public enum TurretState {Spin, ReverseSpin, Reset, Stop, ninety, fortyfive, zero, negfortyfive, negninety, limeylight}
  private final TalonFX Shooter1 = Constants.Shooter1;
  private final TalonFX Shooter2 = Constants.Shooter2;
  private final TalonSRX Turret = Constants.Turret;
  private final Orchestra Music = Constants.TalonMusic;
  NetworkTable limelighty = NetworkTableInstance.getDefault().getTable("limelight-lime");
  NetworkTableEntry tyraw = limelighty.getEntry("ty");
  NetworkTableEntry tvraw = limelighty.getEntry("tv");
  NetworkTableEntry txraw = limelighty.getEntry("tx");
  NetworkTableEntry taraw = limelighty.getEntry("ta");

  double target = tvraw.getDouble(0.0);
  double x = txraw.getDouble(0.0);
  public int max_music = 4;
  public int current_music = 4;
  public Shooter() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setPower(double power)
  {
    Shooter1.set(ControlMode.PercentOutput, power);
    Shooter2.set(ControlMode.PercentOutput, power);
  }

  public void setencpos(double position)
  {
    Turret.set(ControlMode.PercentOutput, 0);
    Turret.setSelectedSensorPosition(position);
  }
  public void angleposition(double desired_angle)
  {
    double angletocounts = desired_angle*67;

    Turret.set(ControlMode.Position, angletocounts);
  }
  
  public void setTurretPower(double power)
  {
    Turret.set(ControlMode.PercentOutput, power);
  }

  public double currentpos()
  {
    return Turret.getSelectedSensorPosition();
  }

  public void limey()
  {
    Timer county = new Timer();
    county.start();
    if(target == 1)
    {
      double offset = currentpos() + x;

      angleposition(offset);

      county.delay(1);
      
    }
  }
  public void loadmusic(int music)
  {

    Music.addInstrument(Shooter1);
    Music.addInstrument(Shooter2);
  
    if(music == 1)
    {
      Music.loadMusic("NyanCat.chrp");
    }
    else if(music == 2)
    {
      Music.loadMusic("RenaiCirculation.chrp");
      
    }
    else if(music == 3)
    {
      Music.loadMusic("Megalovania.chrp");
    }
  }

  public void playmusic()
  {
    Music.play();
  }


}
