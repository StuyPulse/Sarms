package com.stuypulse.robot.commands.Arm;

import com.stuypulse.robot.subsystems.armSim;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class armDrive extends CommandBase{
    private final armSim arm; 
    private final Gamepad gamepad;
    public armDrive(armSim arm, Gamepad gamepad){
        this.arm = arm;
        this.gamepad = gamepad;

        addRequirements(arm);
    }
    @Override
    public void execute(){
        SmartDashboard.putNumber("gamepad rightX", gamepad.getRightX());
        arm.setAngle(arm.getAngleDegrees() + gamepad.getRightX());
    }
    
}
