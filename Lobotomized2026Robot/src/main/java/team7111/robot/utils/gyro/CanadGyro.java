package team7111.robot.utils.gyro;

import 

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public class CanadGyro implements GenericGyro {
    

    private Rotation3d gyroValues = new Rotation3d();
    private Rotation2d yawOffset = Rotation2d.kZero;
    private Rotation2d pitchOffset = Rotation2d.kZero;
    private Rotation2d rollOffset = Rotation2d.kZero;

    private double yawInversion = 1;
    private double pitchInversion = 1;
    private double rollInversion = 1;

    private double permaOffset = 0;
}
