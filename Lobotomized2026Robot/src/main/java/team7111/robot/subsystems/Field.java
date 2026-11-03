package team7111.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Field extends SubsystemBase {
    
    private BooleanSupplier isRedAlliance;

    public Field(BooleanSupplier isRedAlliance){
        this.isRedAlliance = isRedAlliance;
    }

   

    
}
