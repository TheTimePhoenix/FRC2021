package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotPreferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Drive extends SubsystemBase {

    public final DifferentialDrive robotDrive = Constants.robotDrive;

    public enum EncoderEnum { LEFT, RIGHT, MIDDLE, RAWLEFT, RAWRIGHT, RAWMIDDLE };
    
    public enum M_getZeroedEncoder{right, left, average, inch}
    double offset1 = 0;
    double offset2 = 0;
    double offsetinch = 0;
    public int commandflag = 0;
    public double startpoistion = 0;

    public Timer period = new Timer();
    int flug = 0;

    private double leftEncoderOffset = 0;
    private double rightEncoderOffset = 0;

    public double EncoderCountstoInch = .5;

    //COUNTS_PER_INCH = ( COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION ) / ( WHEEL_DIAMETER_INCHES * 3.1415 ) ;

    private boolean useInches = false;


    public Drive() {
        
        SmartDashboard.putBoolean("Rotate PID Tune", false);
        SmartDashboard.putNumber("Rotate P: ", RobotPreferences.getRotateP());
        SmartDashboard.putNumber("Rotate I: ", RobotPreferences.getRotateI());
        SmartDashboard.putNumber("Rotate D: ", RobotPreferences.getRotateD());
        
    //    SmartDashboard.putBoolean("DriveDist PID Tune", false);
    //    SmartDashboard.putNumber("DriveDist P: ", RobotPreferences.getDriveDistP());
    //    SmartDashboard.putNumber("DriveDist I: ", RobotPreferences.getDriveDistI());
    //    SmartDashboard.putNumber("DriveDist D: ", RobotPreferences.getDriveDistD());

    SmartDashboard.putNumber("Heading", Constants.imu.getYaw());

    zeroEncoder(EncoderEnum.RAWMIDDLE);
    zeroEncoder(EncoderEnum.MIDDLE);
    

    }

   


    @Override
    public void periodic() {

        SmartDashboard.putNumber("Encoder Left", getEncoder(EncoderEnum.LEFT));
        SmartDashboard.putNumber("Encoder Right", getEncoder(EncoderEnum.RIGHT));
        SmartDashboard.putNumber("Encoder Conversion", Constants.leftEncoder.getPositionConversionFactor());
        SmartDashboard.putNumber("Encoder CPR", Constants.leftEncoder.getCountsPerRevolution());
        SmartDashboard.putNumber("Raw Encoder Left", Constants.leftEncoder.getPosition());
        SmartDashboard.putNumber("Encoders Counts to Inch", CountstoInch(0));
        SmartDashboard.putNumber("Yaw", getYaw());
        SmartDashboard.putNumber("NavX Pitch", getRoll());

        if (Robot.robotContainer.driveStick.getRawButton(6)) {
            zeroEncoder(EncoderEnum.MIDDLE);
        }

        if (Robot.robotContainer.driveStick.getRawButton(5)) {
            zeroYaw();
        }


        if (SmartDashboard.getBoolean("Rotate PID Tune", false)) {

            updateRotatePID();
        }


        Constants.m_odometry.update(Rotation2d.fromDegrees(getYaw()), inchToMeter(getEncoder(EncoderEnum.RAWLEFT)), inchToMeter(getEncoder(EncoderEnum.RAWRIGHT)));

    }



    public void setDrive(double vY, double vRot) {
        robotDrive.arcadeDrive(vY, vRot);

    }

    public void setTurn(double speed)
    {
        double lspeed = speed*-1;
        double rspeed = speed;
        robotDrive.tankDrive(lspeed, rspeed);
    }

    public void ImuTurn (double speed, double desired_angle)
    {
       // Constants.imu.zeroYaw();
        
        double angleError = getError(desired_angle);
        int speeddirection = -1;


        if(angleError < 1)
        {
            speeddirection = 1;
        }
            
                if(Math.abs(angleError) > 25)
                {
                    speed = speed;
                }    
                else
                {
                    speed = (Math.abs(angleError)/25)*speed*.8;

                    if(speed < .2)
                    {
                        speed = .2;
                    } 
                }
                robotDrive.tankDrive(-speed*speeddirection, speed*speeddirection);
                SmartDashboard.putNumber("angleError", getError(desired_angle));
                SmartDashboard.putNumber("Current Angle", Constants.imu.getYaw());
                
            
       
    
    }
    public double CountstoInch(double state)
    {
        double value = 0;
        if(state == 0)
        {
            value = Constants.leftEncoder.getPosition();
        }
        else if(state == 1)
        {
            value = Constants.leftEncoder.getPosition() - startpoistion;
        }
        else if(state == 2)
        {
            value = Constants.rightEncoder.getPosition()*-1;
        }
        else if(state == 3)
        {
            value = (Constants.rightEncoder.getPosition()*-1) - startpoistion;
        }
        else if(state == 4)
        {
            value = ((Constants.rightEncoder.getPosition()*-1) + Constants.leftEncoder.getPosition())/2;
        }
        else if(state == 5)
        {
            value = ((Constants.rightEncoder.getPosition()*-1) + Constants.leftEncoder.getPosition())/2 - startpoistion;
        }
        return value;
    }

    public double getSteer(double error, double PCoeff) {
        double value = 0;
        if(Math.abs(error*PCoeff) > .3)
        {
            if(error < 0)
            {
                value = -.3;
            }
            else
            {
            value = .3;
            }
        }
        else
        {
            value = error*PCoeff;
        }
        return value;
    }

    
    public double getError(double targetAngle) {

        double robotError;
        double angles;
        
        //rates  = robot.gyro.getAngularVelocity(AngleUnit.DEGREES);
        angles = Constants.imu.getYaw();
        // heading = angles.firstAngle;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - Constants.imu.getYaw();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    

    public void ImuDrive(double distance, double angle, double speed)
    {
       double target = distance;
       double flug = 0;
       double error = 0;
       double steer = 0;
       double commandalreadyexcepted = 0;
       double speeddirection = 1;
       double leftspeed = 0;
       double rightspeed = 0;

       error = getError(angle);
       steer = getSteer(error, .005);

               if(target>CountstoInch(5))
               {
                   speeddirection = -1;
               }
        
            if(Math.abs(target-CountstoInch(5)) > 20)
            {
                 
                speed = speed;
            } 
            else if(Math.abs(target-CountstoInch(5)) < 20)
            {
                speed = ((Math.abs(target-CountstoInch(5)))/20)*speed;

                if(speed < .2)
                {
                    speed = .2;
                }
            }

            if(speeddirection == -1)
            {
                leftspeed = speed - steer;
                rightspeed = speed + steer;
            }
            else
            {
                leftspeed = speed + steer;
                rightspeed = speed - steer;
            }
            
            SmartDashboard.putNumber("Distance Traveled", CountstoInch(1));
            SmartDashboard.putNumber("Timeout", period.get());
            SmartDashboard.putNumber("Steer", steer); 
            SmartDashboard.putNumber("Error IMU_DRIVE", error);
            SmartDashboard.putNumber("Average Encoder", CountstoInch(5));
            robotDrive.tankDrive(leftspeed*speeddirection, rightspeed*speeddirection);
           

           }


     public void LoopEncoderDrive(double speed, double inner_circle_radius, int direction, double full_circle, double distance_offset, double radius_offset)
     {
        double target  = (inner_circle_radius*3.1415*2)*full_circle + distance_offset;
        double position = 0;
        double outer_circle_radius = inner_circle_radius + 22 + radius_offset;
        double new_speed =  outer_circle_radius/inner_circle_radius*speed;


        if(direction == 0 || direction == 3)
        {
            position = CountstoInch(3);
        }
        else
        {
            position = CountstoInch(1);
        }

        if(direction == 2 || direction == 3)
        {
            target = target*-1 ;
        }

        
       
              double flug = 0;
              double commandalreadyexcepted = 0;
              double speeddirection = 1;
       
                      if(target>position)
                      {
                          speeddirection = -1;
                      }
               
                   if(Math.abs(target-position) > 10)
                   {
                        
                       speed = speed;
                       new_speed = outer_circle_radius/inner_circle_radius*speed;
                   } 
                   else if(Math.abs(target-position) < 10)
                   {
                       speed = ((Math.abs(target-position))/10)*speed;
                       new_speed = outer_circle_radius/inner_circle_radius*speed;
       
                       if(speed < .15)
                       {
                           speed = .15;
                           new_speed = outer_circle_radius/inner_circle_radius*speed;
                       }
                   }

                   

                   if(direction == 0)
                   {
                    robotDrive.tankDrive(new_speed*speeddirection, speed*speeddirection);
                   }
                   else if(direction == 1)
                   {
                    robotDrive.tankDrive(speed*speeddirection, new_speed*speeddirection);
                   }
                   else if(direction == 2)
                   {
                    robotDrive.tankDrive(speed*speeddirection, new_speed*speeddirection);
                   }
                   else if(direction == 3)
                   {
                    robotDrive.tankDrive(new_speed*speeddirection, speed*speeddirection);
                   }
                   
                   SmartDashboard.putNumber("Distance Traveled", position);
                   SmartDashboard.putNumber("Timeout", period.get()); 
                  
                  }
      public void EncoderDrive(double distance, double speed)
      {
            double target = distance;
            double flug = 0;
            double commandalreadyexcepted = 0;
            double speeddirection = 1;
                    
            if(target>CountstoInch(1))
             {
                speeddirection = -1;
             }
                            
            if(Math.abs(target-CountstoInch(5)) > 10)
             {
                                     
                speed = speed;
             } 
            else if(Math.abs(target-CountstoInch(5)) < 10)
             {
                 speed = ((Math.abs(target-CountstoInch(1)))/10)*speed;
                    
                   if(speed < .2)
                   {
                        speed = .2;
                   }
             }
                                
                 SmartDashboard.putNumber("Distance Traveled", CountstoInch(1));
                 SmartDashboard.putNumber("Timeout", period.get()); 
                 robotDrive.tankDrive(speed*speeddirection, speed*speeddirection);
                               
                }
           


    

    public double getAverageInchPosition(double state)
    {
        double cpi = 0;
        if(state == 0)
        {
        double averageenc = (Constants.leftEncoder.getPosition() + Constants.rightEncoder.getPosition())/2;
 
       cpi = averageenc*EncoderCountstoInch;
        }
        else
        {
            double averageenc = (Constants.leftEncoder.getPosition() + Constants.rightEncoder.getPosition())/2;
 
            cpi = averageenc*EncoderCountstoInch - getZeroedEncoder(M_getZeroedEncoder.inch);
        }

        return cpi;
    }

    public double getZeroedEncoder (M_getZeroedEncoder state)
    {
        switch(state)
         {
           case left:
           offset1 = Constants.leftEncoder.getPosition();
           return Constants.leftEncoder.getPosition() - offset1;
          
           case right:
           offset2 = Constants.rightEncoder.getPosition();
           return Constants.rightEncoder.getPosition() - offset1;
       
           case average:
           double main = getZeroedEncoder(M_getZeroedEncoder.right) + getZeroedEncoder(M_getZeroedEncoder.left)/2;
           return main;
         
         }
         return 0;
    }
    
    

    public void setStraightDrive(double vY) {

        double leftEncoder = getEncoder(EncoderEnum.LEFT);
        double rightEncoder = getEncoder(EncoderEnum.RIGHT);

        double leftPower = vY;
        double rightPower = vY;

        if (leftEncoder > rightEncoder) {
            leftPower *= 0.9;
            rightPower *= 1.0;
        }
        else if (rightEncoder > leftEncoder) {
            leftPower *= 1.0;
            rightPower *= 0.9;
        }

        SmartDashboard.putNumber("Straight Drive Left", leftPower);
        SmartDashboard.putNumber("Straight Drive Right", rightPower);


        robotDrive.tankDrive(leftPower, rightPower);
    }

    public void setTankVoltage(double leftVolts, double rightVolts) {
        Constants.m_left.setVoltage(leftVolts);
        Constants.m_right.setVoltage(rightVolts);
        Constants.robotDrive.feed();
    }

    public double getDrivePower(){
        return (Constants.spark1.get()+Constants.spark4.get())/2;
    }

    public Pose2d getPose() {
        return Constants.m_odometry.getPoseMeters();
      }

    public void resetOdometry(Pose2d pose) {
        zeroEncoder(EncoderEnum.RAWLEFT);
        zeroEncoder(EncoderEnum.RAWRIGHT);
        Constants.m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getYaw()));
    }

    public double getEncoder(EncoderEnum encoder) {
        switch (encoder) {
        case LEFT:
            return -Constants.leftEncoder.getPosition()-leftEncoderOffset;
        case RIGHT:
            return Constants.rightEncoder.getPosition()-rightEncoderOffset;
        case MIDDLE:
            return (getEncoder(EncoderEnum.LEFT)+getEncoder(EncoderEnum.RIGHT))/2;
        case RAWLEFT:
            return -Constants.leftEncoder.getPosition();
        case RAWRIGHT:
            return Constants.rightEncoder.getPosition();
        case RAWMIDDLE:
            return (getEncoder(EncoderEnum.RAWLEFT)+getEncoder(EncoderEnum.RAWRIGHT))/2;
        }

        return 0;
    }

    public double getAverageEncoder() {
        return getEncoder(EncoderEnum.MIDDLE);
    }

    public void zeroEncoder(EncoderEnum encoder) {
        switch (encoder) {
            case LEFT:
                leftEncoderOffset = getEncoder(EncoderEnum.RAWLEFT);
                break;
            case RIGHT:
                rightEncoderOffset = getEncoder(EncoderEnum.RAWRIGHT);
                break;
            case MIDDLE:
                zeroEncoder(EncoderEnum.LEFT);
                zeroEncoder(EncoderEnum.RIGHT);
                break;
            case RAWLEFT:
                Constants.leftEncoder.setPosition(0);
                leftEncoderOffset = 0;
                break;
            case RAWRIGHT:
                Constants.rightEncoder.setPosition(0);
                rightEncoderOffset = 0;
                break;
            case RAWMIDDLE:
                zeroEncoder(EncoderEnum.RAWLEFT);
                zeroEncoder(EncoderEnum.RAWRIGHT);
                break;
        }
            
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return (new DifferentialDriveWheelSpeeds(Constants.leftEncoder.getVelocity(), Constants.leftEncoder.getVelocity()));
      }

    public double getYaw() {
        return Constants.imu.getYaw();
    }

    public double getRoll() {
        return Constants.imu.getRoll();
    }

    public double getTurnRate() {
        return Constants.imu.getRate();
    }

    public void zeroYaw() {
        Constants.imu.zeroYaw();
    }

    public double inchToMeter(double inch) {
        return inch/39.37;
    }

    public double meterToInch(double meter) {
        return meter*39.37;
    }

    public void useInches(boolean useInches) {
        this.useInches = useInches;
    }

    public void configureUseInches() {
        if (useInches) {
            Constants.leftEncoder.setPositionConversionFactor(Constants.encoderConversionInches);
            Constants.rightEncoder.setPositionConversionFactor(Constants.encoderConversionInches);
        }
        else {
            Constants.leftEncoder.setPositionConversionFactor(Constants.encoderConversionMeters);
            Constants.rightEncoder.setPositionConversionFactor(Constants.encoderConversionMeters);
        }
    }

    public void updateRotatePID() {
        RobotPreferences.setRotateP(SmartDashboard.getNumber("Rotate P: ", 0.0));
        RobotPreferences.setRotateI(SmartDashboard.getNumber("Rotate I: ", 0.0));
        RobotPreferences.setRotateD(SmartDashboard.getNumber("Rotate D: ", 0.0));
    }

    public void updateDriveDistPID() {
        RobotPreferences.setDriveDistP(SmartDashboard.getNumber("DriveDist P: ", 0.0));
        RobotPreferences.setDriveDistI(SmartDashboard.getNumber("DriveDist I: ", 0.0));
        RobotPreferences.setDriveDistD(SmartDashboard.getNumber("DriveDist D: ", 0.0));
    }

}

