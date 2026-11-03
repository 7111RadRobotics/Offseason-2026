package team7111.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team7111.lib.pathfinding.Path;
import team7111.lib.pathfinding.Waypoint;
import team7111.lib.pathfinding.WaypointConstraints;
import team7111.robot.subsystems.SuperStructure.SuperState;
import team7111.robot.utils.AutoAction;

public class Autonomous extends SubsystemBase {

    private Timer timer = new Timer();


    private StructArrayPublisher<Pose2d> hubPublisher = 
            NetworkTableInstance.getDefault().getStructArrayTopic("Hub Presets", Pose2d.struct).publish();

    private WaypointConstraints fastTransConstraints = new WaypointConstraints(8, 0, 3, 1);
    private WaypointConstraints fastRotConstraints = new WaypointConstraints(720, 0, 360, 90);
    
    private WaypointConstraints balancedTransConstraints = new WaypointConstraints(4, 0, 3, 0.25);
    private WaypointConstraints balancedRotConstraints = new WaypointConstraints(540, 0, 270, 5);

    private WaypointConstraints slowTransConstraints = new WaypointConstraints(1.5, 0, 3, 0.1);
    private WaypointConstraints slowRotConstraints = new WaypointConstraints(360, 0, 180, 0.8);

    private SendableChooser<Autos> autoChooser = new SendableChooser<>();
    private SuperStructure superStructure;

    private Pose2d assumedStartingPose = new Pose2d(0, 0, Rotation2d.kZero);

    

  

    //starting position abreviated. separated with "_"
    
    /**
     * {@link Autonomous#getAutonomous(Autos)}
     */
    public enum Autos {
        forwardTest,
        defaultAuto,
    
    }

    /**
     * {@link Autonomous#getPath(Paths)}
     */
    public enum Paths {
        forward,
        defaultPath,
    }

    public Autonomous(){
        Autos defaultAuto = Autos.defaultAuto;
        for (Autos auto : Autos.values()) {
            autoChooser.addOption(auto.name(), auto);
        }
     
        
        autoChooser.setDefaultOption(defaultAuto.name(), defaultAuto);
        
        Shuffleboard.getTab("Autonomous").add("AutoChooser", autoChooser);
        Shuffleboard.getTab("Autonomous").addString("Selected Auto", () -> getSelectedAuto().name());

    }

    public void periodic(){
        
    }

    public void simulationPeriodic(){}

    /**
     * 
     * @param autoName
     * @return
     * {@link Autos}
     */
    public ArrayList<AutoAction> getAutonomous(Autos autoName){
        ArrayList<AutoAction> auto = new ArrayList<>();
        // define each autonomous using a switch statement.
        // each auto is an array of "AutoAction's"
        switch (autoName) {
            case forwardTest:
    
                auto.add(new AutoAction(getPath(Paths.forward)));
                
                break;
            case defaultAuto:

                auto.add(new AutoAction(getPath(Paths.defaultPath)));
           
            default:
                break;
        }

       
        return auto;
    }

    /**
     * 
     * @param path
     * @return
     * {@link Paths}
     */
    public Path getPath(Paths path){
        List<Waypoint> waypoints = new ArrayList<>();
        // define Path object for each Paths enum using a switch statement
        switch (path) {
            case forward:
                waypoints.add(slowPoint(1.5, 0.0, 180));
                waypoints.add(slowPoint(0, 0, 0));
                
                break;
            
            case defaultPath:
                waypoints.add(balancedPoint(0, 0, 0));

            default:
                break;
        }
        return new Path(waypoints);
    }

    public Waypoint fastPoint(double x, double y, double rotDegrees) {
       return new Waypoint(new Pose2d(x, y, Rotation2d.fromDegrees(rotDegrees)), fastTransConstraints, fastRotConstraints);
    }

    public Waypoint balancedPoint(double x, double y, double rotDegrees) {
       return new Waypoint(new Pose2d(x, y, Rotation2d.fromDegrees(rotDegrees)), balancedTransConstraints, balancedRotConstraints);
    }

    public Waypoint slowPoint(double x, double y, double rotDegrees) {
       return new Waypoint(new Pose2d(x, y, Rotation2d.fromDegrees(rotDegrees)), slowTransConstraints, slowRotConstraints);
    }

    public Autos getSelectedAuto(){
        return autoChooser.getSelected();
    }

    
   
    public void giveResources(SuperStructure superStructure){
        this.superStructure = superStructure;
    }

    public Pose2d getAssumedPose(){
        return assumedStartingPose;
    }

    private boolean timeDelay(int delay){
        timer.start();
        if (timer.hasElapsed(delay)) {
            timer.reset();
            timer.stop();
            return true;
        }
        return false;
    }
}
