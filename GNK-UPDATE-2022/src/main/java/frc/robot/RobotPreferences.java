package frc.robot;

import edu.wpi.first.wpilibj.Preferences;

public class RobotPreferences {



    public static double getRotateP() {
        return Preferences.getDouble("Rotate kP", 0.0);
    }

    public static double getRotateI() {
        return Preferences.getDouble("Rotate kI", 0.0);
    }

    public static double getRotateD() {
        return Preferences.getDouble("Rotate kD", 0.0);
    }

    public static double getDriveDistP() {
        return Preferences.getDouble("DriveDist kP", 0.0);
    }

    public static double getDriveDistI() {
        return Preferences.getDouble("DriveDist kI", 0.0);
    }

    public static double getDriveDistD() {
        return Preferences.getDouble("DriveDist kD", 0.0);
    }









    public static void setRotateP(double kP) {
        Preferences.putDouble("Rotate kP", kP);
    }

    public static void setRotateI(double kI) {
        Preferences.putDouble("Rotate kI", kI);
    }

    public static void setRotateD(double kD) {
        Preferences.putDouble("Rotate kD", kD);
    }

    public static void setDriveDistP(double kP) {
        Preferences.putDouble("DriveDist kP", kP);
    }

    public static void setDriveDistI(double kI) {
        Preferences.putDouble("DriveDist kI", kI);
    }

    public static void setDriveDistD(double kD) {
        Preferences.putDouble("DriveDist kD", kD);
    }
}