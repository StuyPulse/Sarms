package com.stuypulse.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Simulation extends SubsystemBase{


  public Simulation(){
    
  }

  private static final double m_armMass = Units.lbsToKilograms(11); // Kilograms
  private static final double m_armLength = Units.inchesToMeters(20.5);

  private static final double m_gear = 9;


  // gearing =  9:1
  // moment of inertia = 
  // arm length =  20.5 inches
  // min 0 max 90
  // mass : 11 lbs
    private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
      DCMotor.getNEO(1)
      , m_gear
      , SingleJointedArmSim.estimateMOI(m_armLength, m_armMass)
      , m_armLength
      , Units.degreesToRadians(0)
      , Units.degreesToRadians(90)
      , m_armMass
      ,true);

      Mechanism2d arm = new Mechanism2d(3, 3);

      @Override
      public void simulationPeriodic() {
        armSim.setInput(6);
        armSim.update(0.02);
      }
}

