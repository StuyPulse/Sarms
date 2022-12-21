package com.stuypulse.robot.subsystems;

import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class armSim extends SubsystemBase {

  private final SingleJointedArmSim armSim;
  private final MechanismLigament2d armArm;
  private final SmartNumber driverState;

  private final double m_armMass = Units.lbsToKilograms(11); // Kilograms
  private final double m_armLength = Units.inchesToMeters(20.5);
  private static final double m_gear = 9;

  // private final Controller controller;
  private final SmartNumber targetAngle;



  // gearing =  9:1
  // moment of inertia = 
  // arm length =  20.5 inches
  // min 0 max 90
  // mass : 11 lbs
    public armSim() {

      setSubsystem("Arm");
      
      armSim =
      new SingleJointedArmSim(
      DCMotor.getNEO(1)
      , m_gear
      , SingleJointedArmSim.estimateMOI(m_armLength, m_armMass)
      , m_armLength
      , Units.degreesToRadians(0)
      , Units.degreesToRadians(90)
      , m_armMass
      ,true);

      driverState = new SmartNumber("Arm/Driver Simulation State", 0);

      Mechanism2d arm = new Mechanism2d(2, 2);
      MechanismRoot2d armRoot = arm.getRoot("Intake Root", 1, m_armLength);
      armArm = new MechanismLigament2d("Arm Arm", 1, 45);
      armRoot.append(armArm);
      addChild("Arm Mechanism2d", arm);

      targetAngle = new SmartNumber("Arm/Target Angle", 0);

      SmartDashboard.putData(arm);
    }

      @Override
      public void simulationPeriodic() {
        armSim.update(0.02);
        armSim.setInputVoltage(6);

        armSim.setInput(6 * targetAngle.get());

        RoboRioSim.setVInCurrent(BatterySim.calculateDefaultBatteryLoadedVoltage(
                armSim.getCurrentDrawAmps()));
      }

      public void reset (double degrees){
        armSim.setState(VecBuilder.fill(degrees,0));
      }

      public void stop(){
        driverState.set(0);
      }

      public void setAngle(double angle){
        targetAngle.set(angle);
      }

      // public void add

      // public double getTargetAngle(){
      //   return targetAngle.get();
      // }

      public double getAngleDegrees(){
        return armSim.getOutput(0);
      }

      

      
}

