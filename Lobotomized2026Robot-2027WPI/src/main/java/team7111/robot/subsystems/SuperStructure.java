package team7111.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import javax.lang.model.element.ModuleElement.DirectiveKind;
import javax.print.attribute.standard.RequestingUserName;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.Units;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.framework.RobotBase;
import org.wpilib.system.Timer;
import org.wpilib.driverstation.XboxController;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.GenericHID.RumbleType;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.command2.SubsystemBase;

import team7111.robot.subsystems.Autonomous.Autos;
import team7111.robot.subsystems.Autonomous.Paths;
import team7111.robot.subsystems.Swerve.SwerveState;
import team7111.robot.utils.AutoAction;
import team7111.robot.utils.Camera;
import team7111.lib.pathfinding.*;

/**
 * This class handles the overall state of the robot.
 * It does this by defining various "SuperState" values and calling the methods assosiated with each one.
 * Each method uses logic to determine subsystem states, when to switch SuperStates, and what SuperState to go to next.
 * Logic used can be button inputs, subsystem states, or other subsystem conditions.
 */
public class SuperStructure extends SubsystemBase {
    /**
     * This enumeration contains the values or "states" which determine the subsystems.
     * The top 2 state names are temporary states for testing 
     */
    public enum SuperState {
        defaultState,
        autonomousEnter,
        autonomous,
        autonomousExit,
    }

    //Subsystem Variables
    private final Autonomous auto;
    private final Swerve swerve;
    private final Vision vision;
    private final Field field;

    // Buttons of controllers can be assigned to booleans which are checked in various super states. 
    private final XboxController driverController = new XboxController(0);
    private final XboxController operatorController = new XboxController(1);

    /** This represents the current superstate of the robot */
    private SuperState superState = SuperState.defaultState;
    private boolean hasAcheivedState = false;

    private boolean inAuto = false;
    private int autoIndex = 0;
    private List<AutoAction> autoActions;



    private Autos currentAutos;

    private boolean operatorDisabled = false;



    /**
     * The constructor will take each subsystem as an argument and save them as objects in the class. 
     * @param subsystem represents a subsystem. 
     */
    public SuperStructure(Autonomous auto, Swerve swerve, Vision vision, Field field){
        this.auto = auto;
        this.swerve = swerve;
        this.vision = vision;
        this.field = field;
        currentAutos = auto.getSelectedAuto();
        autoActions = auto.getAutonomous(auto.getSelectedAuto());

        DriverStation.silenceJoystickConnectionWarning(true);
        this.swerve.setJoysickInputs(() -> -driverController.getLeftX(), () -> -driverController.getLeftY(), () -> driverController.getRightX());
        this.swerve.setDriveFieldRelative(true);
        this.swerve.setSwerveState(SwerveState.manual);


        auto.giveResources(this);

    }

    /**
     * This method runs every iteration (every 20ms). Actions like state management and stateless logic are run here.
     */
    public void periodic(){
      
        
        //Timer for the periodic
        long startTime = System.nanoTime();
        SmartDashboard.putString("SuperState", superState.name());
        NetworkTable.


       

        SmartDashboard.putBoolean("roboPoseIsNull", vision.getRobotPose(0.1) == null);
        final boolean useOneCamera = false; // determine whether true or false
        boolean hasAppliedVisionMeasurement = false;
        Pose3d visionRobotPoseExample = vision.getRobotPose(vision.exampCam, 0.15);
        Pose3d bestVisionRobotPose = vision.getBestRobotPose(0.15);

        boolean useAssumedPose = true;
        if(bestVisionRobotPose != null){
            useAssumedPose = false;
            swerve.addVisionMeasurement(bestVisionRobotPose.toPose2d(), true);
        }

      
        SmartDashboard.putString("SuperStructure Auto", currentAutos.name());

        // Driver controller commands
      
        if(driverController.getStartButton()) {
            swerve.zeroGyro();
          
        }

        if(driverController.getBackButtonPressed()) {
            swerve.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)));
        }

     
        long endTime = System.nanoTime();

        //Timing measurement, in milliseconds
        SmartDashboard.putNumber("Time for superstructure periodic", (double) ((endTime - startTime) / 1000000.0));

    }

    /**
     * This method is run every iteration (20ms) only in simulation mode. Can be used to update simulated mechanisms or sensors.
     */
    public void simulationPeriodic(){}

    /**
     * This is called the state manager. It checks the value of the SuperState argument and calls the method associated with it.
     * <p>Each state will have its own case statement, returning its state method.
     * @param state the SuperState value
     * @return the condition of the state determined by the state method.
     */
    private boolean manageSuperState(SuperState state){
        switch(state){
            case autonomous:
                return autonomous();
            case autonomousEnter:
                return autonomousEnter();
            case autonomousExit:
                return autonomousExit();
            
            default:
                return defaultState(state);
        }
    }

    /**
     * Each of these methods, called "state methods", represent a defined state.
     * When called by the state manager, it will set the states of different subsystems.
     * @return true if the state is complete. The condition could represent mechanisms at a setpoint, a beambreak trigger, a timer, etc.
     * Mainly used for autonomous routines.
     */
   

    /** 
     * The state of the robot when autonomous initializes.
     * <p>It will reset/initialize certain variables and set the auto to the selected list of {@link AutoAction}'s from the auto chooser in {@link Autonomous}
     * before switching to the autonomous state. It also calls the autonomous state so it does not have to wait an extra iteration
     * @return true since there is no logic to compute
     */
    private boolean autonomousEnter(){
        setSuperState(SuperState.autonomous);
        autoIndex = 0;
        autonomous();
        return true;
    }

    /**
     * The state of the robot during the full autonomous period.
     * This will run each {@link AutoAction} from the list received from {@link Autonomous} in order.
     * <p>If there is an alternative condition in the AutoAction, this method should increment the autoIndex variable after the condition is true.
     * <p>Otherwise, if the AutoAction is a {@link Path}, this method should increment the variable when the path is finished;
     * if it's a {@link SuperState}, this method should increment the variable when the SuperState's state method returns true 
     * @return true if autoIndex >= the size of the AutoAction List
     */
    private boolean autonomous(){
        inAuto = true;
        boolean isActionFinished = true;
        AutoAction autoAction;
        autoAction = autoActions.get(autoIndex);

        SmartDashboard.putNumber("AutoAction", autoActions.size() - 1);
        SmartDashboard.putNumber("autoIndex", autoIndex);
        SmartDashboard.putBoolean("IsPathFinished", isActionFinished);

        if (autoAction.isPath()){
            SwerveState swerveState = swerve.getSwerveState();
            if (!swerveState.equals(SwerveState.initializePath) && !swerveState.equals(SwerveState.runPath)){
                swerve.setPath(autoAction.getAsPath());
                swerve.setSwerveState(SwerveState.initializePath);
            }
            isActionFinished = autoAction.getAsPath().isPathFinished();
        } else if (autoAction.isState()) {
            isActionFinished = manageSuperState(autoAction.getAsState());
        }

        if (autoAction.hasAlternateCondition()){
            isActionFinished = autoAction.getAlternateCondition();
        }

        if (autoAction.hasAdditionalCondition()){
            isActionFinished = (isActionFinished && autoAction.getAdditionalCondition());
        }

        if (autoAction.hasOptionalCondition()){
            isActionFinished = (isActionFinished || autoAction.getOptionalCondition());
        }

        if (isActionFinished) {
            if (autoActions.size() - 1 > autoIndex) {
                autoIndex += 1;
                
            }
            else {
                inAuto = false;
                setSuperState(SuperState.autonomousExit);
            }

            
        }

        return true;
    }

    /**
     * The state of the robot at the end of the autonomous period.
     * <p>This will set the robot up for Teleop control by setting inAuto to false, 
     * setting the swerve state to manual, and changing the superState (using setSuperState()) to a different state
     * @return true 
     */
    private boolean autonomousExit(){
        inAuto = false;
        swerve.setSwerveState(SwerveState.manual);
        setSuperState(SuperState.defaultState);
        
       
        return true;
    }

    /**
     * The method called when the state method of the managed SuperState is not called.
     * @param state
     * @return
     */
    private boolean defaultState(SuperState state){
        System.err.println("The SuperState \"" + state.name() + "\" does not have a state method or it was not called."
            + "\nPlease return the state method in a new case statement in manageState()");
        return true;
    }

    /**
     * This method is responsible for changing the robot's {@link SuperState}.
     * <p> This method MUST be used when changing the robot's SuperState to ensure it does not interfere with autonomous.
     * @param state the SuperState to set the robot if not in Autonomous mode
     */
    public void setSuperState(SuperState state){
    
        if(!inAuto)
            superState = state;
    }

    /**
     * Gets the current {@link SuperState} of the robot.
     * @return the value of the superState object
     */
    public SuperState getSuperState(){
        return superState;
    }

    public void disableAuto(){
        superState = SuperState.autonomousExit;
    }

    
}
