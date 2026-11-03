package team7111.robot.utils.swervemodules;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveModuleState;
import team7111.robot.utils.encoder.GenericEncoder;

public interface GenericSwerveModule {
    public void setOpenDriveState(SwerveModuleState state);

    public void setClosedDriveState(SwerveModuleState state);

    /** Returns in rotations per second */
    public double getDriveVelocity();

    public double getDrivePosition();

    public Rotation2d getAngle();

    public Translation2d getOffset();

    public void setAngle(Rotation2d rotation);

    public GenericEncoder getEncoder();

    public void zeroWheels();

    public void configure();

    default public void update(){}
}
