package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveMath;

public class Robot extends TimedRobot {

  private final SwerveDrive swerveDrive = new SwerveDrive();
  private final XboxController controller = new XboxController(0);

  // later thing
  private final double wheelbase = 0.5;
  private final double trackwidth = 0.5;

  @Override
  public void robotInit() {

    SmartDashboard.putNumber("Test Forward", 0);
    SmartDashboard.putNumber("Test Strafe", 0);
    SmartDashboard.putNumber("Test Rotation", 0);

    // False = manual SmartDashboard
    // True = Xbox controller
    SmartDashboard.putBoolean("Use Xbox Controller", false);

    SwerveMath.WheelState testState = new SwerveMath.WheelState(1.0, Math.toRadians(170));

    SwerveMath.WheelState optimized = SwerveMath.optimize(0, testState);

    SmartDashboard.putNumber("Test Optimized Speed", optimized.speed);

    SmartDashboard.putNumber(
        "Test Optimized Angle",
        Math.toDegrees(optimized.angle));

  }

  @Override
  public void robotPeriodic() {

    // reading of CANcoder
    SmartDashboard.putNumber(
        "Front Left Encoder",
        swerveDrive.getFrontLeftEncoder());

    SmartDashboard.putNumber(
        "Front Right Encoder",
        swerveDrive.getFrontRightEncoder());

    SmartDashboard.putNumber(
        "Back Left Encoder",
        swerveDrive.getBackLeftEncoder());

    SmartDashboard.putNumber(
        "Back Right Encoder",
        swerveDrive.getBackRightEncoder());

    double forward;
    double strafe;
    double rotation;

    boolean useController = SmartDashboard.getBoolean("Use Xbox Controller", false);

    if (useController) {

      forward = getDriveForward();
      strafe = getDriveStrafe();
      rotation = getDriveRotation();

    } else {

      forward = SmartDashboard.getNumber("Test Forward", 0);
      strafe = SmartDashboard.getNumber("Test Strafe", 0);
      rotation = SmartDashboard.getNumber("Test Rotation", 0);
    }

    // joystick values after deadband
    SmartDashboard.putNumber("Forward Input", forward);
    SmartDashboard.putNumber("Strafe Input", strafe);
    SmartDashboard.putNumber("Rotation Input", rotation);

    // Wheel speed and angles
    SwerveMath.WheelState[] wheels = SwerveMath.calculate(
        forward,
        strafe,
        rotation,
        wheelbase,
        trackwidth);
    SwerveMath.WheelState[] optimizedWheels = swerveDrive.optimizeAll(wheels);

    String[] names = {
        "Front Left",
        "Front Right",
        "Back Left",
        "Back Right"
    };

    for (int i = 0; i < 4; i++) {

      SmartDashboard.putNumber(
          names[i] + " Speed",
          wheels[i].speed);

      SmartDashboard.putNumber(
          names[i] + " Angle Degrees",
          Math.toDegrees(wheels[i].angle));

      SmartDashboard.putNumber(
          names[i] + " Optimized Speed",
          optimizedWheels[i].speed);

      SmartDashboard.putNumber(
          names[i] + " Optimized Angle",
          Math.toDegrees(optimizedWheels[i].angle));
    }
  }

  // controller deadband for drift
  private double applyDeadband(double value) {
    if (Math.abs(value) < 0.08) {
      return 0;
    }
    return value;
  }

  public double getDriveForward() {
    return applyDeadband(-controller.getLeftY());
  }

  public double getDriveStrafe() {
    return applyDeadband(-controller.getLeftX());
  }

  public double getDriveRotation() {
    return applyDeadband(-controller.getRightX());
  }
}