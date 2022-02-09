package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.music.Orchestra;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.I2C;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
//import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.util.Color;
//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Servo;

public class Constants {

    public static AHRS imu;
    public static DifferentialDriveOdometry m_odometry;
    public static CANSparkMax spark1;
    public static CANSparkMax spark2;
    public static CANSparkMax spark3;
    public static CANSparkMax spark4;
    public static CANSparkMax spark5;
    public static CANSparkMax spark6;
    public static CANSparkMax Lift1;
    public static CANSparkMax Lift2;
    public static Servo Lift1Servo;
    public static Servo Lift2Servo;
    public static DigitalInput Lift1Up;
    public static DigitalInput Lift1Down;
    public static DigitalInput Lift2Up;
    public static DigitalInput Lift2Down;
    public static SpeedControllerGroup m_left;
    public static SpeedControllerGroup m_right;
    public static DifferentialDrive robotDrive;

    private static final AlternateEncoderType kAltEncType = AlternateEncoderType.kQuadrature;
    private static final int kCPR = 1024;
    public static CANEncoder leftEncoder;
    public static CANEncoder rightEncoder;

    //Characterization Variables
    public static final double ksVolts = 0.177;
    public static final double kvVoltSecondsPerMeter = 0.614;
    public static final double kaVoltSecondsSquaredPerMeter = .142;

    public static final double kPDriveVel = 6.52;

    public static final double kTrackwidthMeters = 0.57785;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;


    public static TalonFX Shooter1;
    public static TalonFX Shooter2;
    public static Orchestra TalonMusic;
    public static TalonSRX Turret;
    public static TalonSRX ShooterRoller;
    public static TalonSRX Roller1;
    public static VictorSPX Roller2;
    public static VictorSPX Intake;
    //public static WPI_TalonSRX intakeMotor;
    public static WPI_TalonSRX cpArm;

    public static ColorSensorV3 colorSensor;
    public static ColorMatch colorMatcher;

    public final static Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    public final static Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    public final static Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    public final static Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

    public static AnalogInput Stage1;
    public static AnalogInput Stage2;
    public static AnalogInput Stage3;
    public static AnalogInput Ramp;

    private static final double driveRampRate = .25;

    public final static double encoderConversionInches = 1 / (.51 / 10); // inverse of ticks per inch
    public final static double encoderConversionMeters = 1 / (2.0078);

    public static void init() {

        imu = new AHRS(SPI.Port.kMXP);

        driveInit();

        funnelInit();

        cpArmInit();

        liftInit();

        shoooterInit();

    }

    static void driveInit() {

        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(imu.getAngle()));

        spark1 = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
        spark1.setInverted(false);
        spark1.setOpenLoopRampRate(driveRampRate);
        spark1.setIdleMode(IdleMode.kBrake);

        spark2 = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
        spark2.setInverted(false);
        spark2.setOpenLoopRampRate(driveRampRate);
        spark2.setIdleMode(IdleMode.kBrake);

        spark3 = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
        spark3.setInverted(false);
        spark3.setOpenLoopRampRate(driveRampRate);
        spark3.setIdleMode(IdleMode.kBrake);

        spark4 = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
        spark4.setInverted(true);
        spark4.setOpenLoopRampRate(driveRampRate);
        spark4.setIdleMode(IdleMode.kBrake);

        spark5 = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
        spark5.setInverted(true);
        spark5.setOpenLoopRampRate(driveRampRate);
        spark5.setIdleMode(IdleMode.kBrake);

        spark6 = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
        spark6.setInverted(true);
        spark6.setOpenLoopRampRate(driveRampRate);
        spark6.setIdleMode(IdleMode.kBrake);

        m_left = new SpeedControllerGroup(spark1, spark2, spark3);

        m_right = new SpeedControllerGroup(spark4, spark5, spark6);

        robotDrive = new DifferentialDrive(m_left, m_right);
        robotDrive.setRightSideInverted(false);
        robotDrive.setSafetyEnabled(false);

        leftEncoder = spark1.getAlternateEncoder(kAltEncType, kCPR);
        leftEncoder.setPositionConversionFactor(6*3.1415);

        rightEncoder = spark4.getAlternateEncoder(kAltEncType, kCPR);
        rightEncoder.setPositionConversionFactor(6*3.1415);

    
       
    }
    static void shoooterInit()
    {
        Shooter1 = new TalonFX(1);
        Shooter2 = new TalonFX(2);
        Turret = new TalonSRX(3);
        ShooterRoller = new TalonSRX(4);
        TalonMusic = new Orchestra();
        Ramp = new AnalogInput(3);

        Shooter1.setNeutralMode(NeutralMode.Coast);
        Shooter2.setNeutralMode(NeutralMode.Coast);
        ShooterRoller.setNeutralMode(NeutralMode.Brake);
        Shooter1.setInverted(true);
        Shooter2.setInverted(false);

        Turret.setNeutralMode(NeutralMode.Coast);
        Turret.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 30);
        Turret.setSensorPhase(true);
        Turret.configNominalOutputForward(0, 30);
        Turret.configNominalOutputReverse(0, 30);
        Turret.configPeakOutputForward(.3, 30);
        Turret.configPeakOutputReverse(-.3, 30);
        Turret.config_kF(0, 0, 30);
        Turret.config_kP(0, 0.6, 30);
        Turret.config_kI(0, 0, 30);
        Turret.config_kD(0, 1,30);
        Turret.configAllowableClosedloopError(0, 0, 30);

        TalonMusic.addInstrument(Shooter1);
        TalonMusic.addInstrument(Shooter2);  
        
        TalonMusic.loadMusic("NyanCat.chrp");
    }

    static void funnelInit() {

        Roller1 = new TalonSRX(5);
        Roller2 = new VictorSPX(6);
        Intake = new VictorSPX(7);

        Roller1.setNeutralMode(NeutralMode.Brake);
        Roller2.setNeutralMode(NeutralMode.Brake);
        Intake.setNeutralMode(NeutralMode.Brake);

        Roller1.setInverted(true);
        Roller2.setInverted(false);
        Intake.setInverted(false);

        Stage1 = new AnalogInput(0);
        Stage2 = new AnalogInput(1);
        Stage3 = new AnalogInput(2);
    }

    static void cpArmInit() {

        cpArm = new WPI_TalonSRX(7);

        cpArm.setNeutralMode(NeutralMode.Brake);
        cpArm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        cpArm.setSensorPhase(true);

        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        colorMatcher = new ColorMatch();

        colorMatcher.addColorMatch(kBlueTarget);
        colorMatcher.addColorMatch(kGreenTarget);
        colorMatcher.addColorMatch(kRedTarget);
        colorMatcher.addColorMatch(kYellowTarget); 


    }
    static void liftInit(){
        Lift1 = new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless);
        Lift1.setInverted(false);
        Lift1.setIdleMode(IdleMode.kBrake);

        Lift2 = new CANSparkMax(8, CANSparkMaxLowLevel.MotorType.kBrushless);
        Lift2.setInverted(true);
        Lift2.setIdleMode(IdleMode.kBrake);

        Lift1Servo = new Servo(1);
        Lift2Servo = new Servo(3);

        Lift1Up = new DigitalInput(0);
        Lift1Down = new DigitalInput(1);
        Lift2Up = new DigitalInput(2);
        Lift2Down = new DigitalInput(3);
      
        
    }

}