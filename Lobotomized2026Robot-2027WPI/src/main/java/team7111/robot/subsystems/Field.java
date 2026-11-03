package team7111.robot.subsystems;

import java.util.function.BooleanSupplier;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.driverstation.Alliance;
import org.wpilib.command2.SubsystemBase;

public class Field extends SubsystemBase {
    
    private BooleanSupplier isRedAlliance;

    public Field(BooleanSupplier isRedAlliance){
        this.isRedAlliance = isRedAlliance;
    }

   

    
}
