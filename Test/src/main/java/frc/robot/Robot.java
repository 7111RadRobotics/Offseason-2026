// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  public enum State {
    State1,
    State2,
    State3,
  }

  public State state = State.State1;

  public Timer timer = new Timer();

  private boolean timeDelay(int delay){
        timer.start();
        if (timer.hasElapsed(delay)) {
            timer.reset();
            timer.stop();
            return true;
        }
        return false;
    }

  public Robot() {
   
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {


      switch(state) {

          case State1:
           
            System.out.println("Current State = " + state.toString());
            if (timeDelay(5)) {
              state = state.State2;
            }
            break;
          case State2:
           
            System.out.println("Current State = " + state.toString());
            if (timeDelay(5)) {
              state = State.State3;
            }
            break;
          case State3:
            timer.start();
            System.out.println("Current State = " + state.toString());
            if (timeDelay(5)) {
            
              state = State.State1;
            }
            break;
      }

      
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
