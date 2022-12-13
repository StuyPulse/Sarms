package com.stuypulse.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Simulation extends SubsystemBase{
  private static final double m_armReduction = 600;
  private static final double m_armMass = 5.0; // Kilograms
  private static final double m_armLength = Units.inchesToMeters(30);


    private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
      DCMotor.getNEO(1)
      , 1
      , SingleJointedArmSim.estimateMOI(m_armLength, m_armMass)
      , m_armLength
      , Units.degreesToRadians(-75)
      , Units.degreesToRadians(255)
      , m_armMass
      ,true);

      @Override
      public void simulationPeriodic() {

      }
}

