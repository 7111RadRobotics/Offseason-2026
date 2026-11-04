package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;

public class SwerveModule {
    private final SparkMax driveMotor;
    private final SparkMax turnMotor;
    private final CANcoder absoluteEncoder;

    public SwerveModule(int driveMotorID, int turnMotorID, int encoderID) {
        driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        turnMotor = new SparkMax(turnMotorID, MotorType.kBrushless);
        absoluteEncoder = new CANcoder(encoderID);
    }

    // how fast wheel goes
    public void setDrivePower(double power) {
        driveMotor.set(power);
    }

    // steering motor thingy
    public void setTurnPower(double power) {
        turnMotor.set(power);
    }

    // both motor stopper
    public void stop() {
        driveMotor.stopMotor();
        turnMotor.stopMotor();
    }

    // encoder posistion reader
    public double getAbsoluteEncoderPosition() {
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    }

    // encoder reading from rotations to radians
    public double getCurrentAngleRadians() {
        return getAbsoluteEncoderPosition() * 2.0 * Math.PI;
    }

    // best turn for wheel
    public SwerveMath.WheelState getOptimizedState(
            SwerveMath.WheelState desiredState) {
        return SwerveMath.optimize(
                getCurrentAngleRadians(),
                desiredState);
    }
}