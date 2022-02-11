package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

    public class Funnel extends SubsystemBase {

        public enum FunnelState {Collect, Remove, Stop};

        private final TalonSRX Roller1 = Constants.Roller1;
        private final VictorSPX Roller2 = Constants.Roller2;
        private final VictorSPX Intake = Constants.Intake;
        private final TalonSRX RampMotor = Constants.ShooterRoller;

        private final AnalogInput Ramp = Constants.Ramp;
        private final AnalogInput Stage1 = Constants.Stage1;
        private final AnalogInput Stage2 = Constants.Stage2;
        private final AnalogInput Stage3 = Constants.Stage3;

        //The voltage value for each ir sensor to detect a ball's presence
        private final double ballVolt1 = 1.5;
        private final double ballVolt2 = 2;
        private final double ballVolt3 = 1.4;
        private final double ballVolt4 = 1.4;
        private final double ballVolt5 = 1.5;

        


        public Funnel () {

        }

        @Override
        public void periodic() {
           SmartDashboard.putNumber("Stage1Volt", getStage1());
           SmartDashboard.putNumber("Stage2Volt", getStage2());
           SmartDashboard.putNumber("Stage3Volt", getStage3());
           SmartDashboard.putNumber("StageRampVolt", getRamp());
        }
        
        public void setRamp(double power)
        {
          RampMotor.set(ControlMode.PercentOutput, -power);
        }
        public void setRoller1(double power)
        {
          Roller1.set(ControlMode.PercentOutput, power); 
        }

        public void setRoller2(double power)
        {
          Roller2.set(ControlMode.PercentOutput, power);
        }

        public void setIntake(double power)
        {
          Intake.set(ControlMode.PercentOutput, -power);
        }

        public double getStage1()
        {
            return Stage1.getAverageVoltage();
        }

        public double getStage2()
        {
            return Stage2.getAverageVoltage();
        }

        public double getStage3()
        {
          return Stage3.getAverageVoltage();
        }

        public double getRamp()
        {
          return Ramp.getAverageVoltage();
        }
    }