package frc.robot.commands;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class StickDrive extends CommandBase {

    private final Drive drive = Robot.drive;

	static final double DEADZONE = .07;

	//private final double veeringDeadzone = 0.1;

	public class JoystickResponseCurve {
		double adjust;
		double power;
		double multiplier;
		double deadzone;

		public JoystickResponseCurve(final double adj, final double pow, final double mult, final double dead) {
			adjust = adj;
			power = pow;
			multiplier = mult;
			deadzone = dead;
		}

		// f(x) = multiplier * (adjust * x^power+(1-adjust)*x)

		public double transform(final double input) {
			double output = 0.0;
			if ((input > deadzone) || (input < (-1 * deadzone))) {
				output = multiplier * (adjust * Math.pow(input, power) + (1 - adjust) * input);
			}
			return output;
		}
	}

	public class JoystickResponseCurveSet {
		JoystickResponseCurve fwd;
		JoystickResponseCurve strafe;
		JoystickResponseCurve rotate;

		public JoystickResponseCurveSet(final JoystickResponseCurve fwd, final JoystickResponseCurve strafe,
				final JoystickResponseCurve rotate) {
			this.fwd = fwd;
			this.strafe = strafe;
			this.rotate = rotate;
		}

		public double transformForward(final double input) {
			return fwd.transform(input);
		}

		public double transformStrafe(final double input) {
			return strafe.transform(input);
		}

		public double transformRotate(final double input) {
			return rotate.transform(input);
		}
	}

	JoystickResponseCurveSet linear = new JoystickResponseCurveSet(new JoystickResponseCurve(.00, 3, 1.0, DEADZONE),
			new JoystickResponseCurve(.00, 0, 0, DEADZONE), new JoystickResponseCurve(.00, 3, 1.0, DEADZONE));

	JoystickResponseCurveSet conservative = new JoystickResponseCurveSet(
			new JoystickResponseCurve(.40, 3, .80, DEADZONE), new JoystickResponseCurve(.40, 3, .80, DEADZONE),
			new JoystickResponseCurve(.30, 3, .60, DEADZONE));

	JoystickResponseCurveSet aggressive = new JoystickResponseCurveSet(new JoystickResponseCurve(.40, 3, 1.0, DEADZONE),
			new JoystickResponseCurve(.40, 3, 1.0, DEADZONE), new JoystickResponseCurve(.40, 3, 1.0, DEADZONE));

	public StickDrive(final Drive drive) {
		addRequirements(drive);
	}

	@Override
	public void initialize() {
		SmartDashboard.putBoolean("StickInit", true);
	}

	@Override
	public void execute() {

		SmartDashboard.putBoolean("Straight Fasty", Robot.robotContainer.fullspeedy.get());

		final JoystickResponseCurveSet slow = conservative;
		final JoystickResponseCurveSet fast = aggressive;

		final Joystick driver = Robot.robotContainer.driveStick;

		double vY = -driver.getY();
		double vRot = 0;
		double joyX = driver.getX();
		double joyRot = driver.getRawAxis(4)*.8575;
		boolean veering = false;

		if(Robot.robotContainer.fullspeedy.get() == true)
		{
		  vY = .75;
		  //joyX = 0;
		  joyRot = 0;
		}
	


		if (Math.abs(joyX) >= Math.abs(joyRot)) {
			vRot = (vY < 0)? joyX*-1:joyX;
			veering = true;
		}
		else {
			vRot = joyRot;
			veering = false;
		}
		


		if (!driver.getRawButton(1)) {
			vY = slow.transformForward(vY);
			vRot = veering? slow.transformStrafe(vRot):slow.transformRotate(vRot);
			SmartDashboard.putBoolean("Slow Drive Profile", true);
		} else {
			vY = fast.transformForward(vY);
			vRot = veering? fast.transformStrafe(vRot):fast.transformRotate(vRot);
			SmartDashboard.putBoolean("Slow Drive Profile", false);
		}

		if (Constants.robotDrive != null) {
			((Drive) drive).setDrive(vY, vRot);
		}


/*
		if (driver.getRawButton(7)) {
			Constants.intakeMotor.set(Math.abs(driver.getRawAxis(2)));
		}
		else if (driver.getRawButton(8)) {
			Constants.intakeMotor.set(-Math.abs(driver.getRawAxis(2)));
		}
		else {
			Constants.intakeMotor.set(0);
		}
*/

	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(final boolean interrupted) {
		drive.setDrive(0, 0);
    }




}