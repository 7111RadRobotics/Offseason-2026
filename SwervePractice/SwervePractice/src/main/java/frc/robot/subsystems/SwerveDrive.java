
package frc.robot.subsystems;

import frc.robot.Constants;

public class SwerveDrive {

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    public SwerveDrive() {

        // Set up each of the four swerve modules.
        frontLeft = new SwerveModule(
                Constants.FRONT_LEFT_DRIVE_ID,
                Constants.FRONT_LEFT_TURN_ID,
                Constants.FRONT_LEFT_CANCODER_ID);

        frontRight = new SwerveModule(
                Constants.FRONT_RIGHT_DRIVE_ID,
                Constants.FRONT_RIGHT_TURN_ID,
                Constants.FRONT_RIGHT_CANCODER_ID);

        backLeft = new SwerveModule(
                Constants.BACK_LEFT_DRIVE_ID,
                Constants.BACK_LEFT_TURN_ID,
                Constants.BACK_LEFT_CANCODER_ID);

        backRight = new SwerveModule(
                Constants.BACK_RIGHT_DRIVE_ID,
                Constants.BACK_RIGHT_TURN_ID,
                Constants.BACK_RIGHT_CANCODER_ID);
    }

    // Stop all four wheels.
    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    // Get the encoder position for each wheel.
    public double getFrontLeftEncoder() {
        return frontLeft.getAbsoluteEncoderPosition();
    }

    public double getFrontRightEncoder() {
        return frontRight.getAbsoluteEncoderPosition();
    }

    public double getBackLeftEncoder() {
        return backLeft.getAbsoluteEncoderPosition();
    }

    public double getBackRightEncoder() {
        return backRight.getAbsoluteEncoderPosition();
    }

    // Figure out the best direction for each wheel to turn.
    public SwerveMath.WheelState[] optimizeAll(
            SwerveMath.WheelState[] desiredStates) {

        return new SwerveMath.WheelState[] {
                frontLeft.getOptimizedState(desiredStates[0]),
                frontRight.getOptimizedState(desiredStates[1]),
                backLeft.getOptimizedState(desiredStates[2]),
                backRight.getOptimizedState(desiredStates[3])
        };
    }
}