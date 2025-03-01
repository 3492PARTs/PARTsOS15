// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class PARTsButtonBoxController {
    private Joystick joystick;

    public PARTsButtonBoxController(int port){
        joystick = new Joystick(port);
    }
   
    public Trigger handleTrigger(){
        return new Trigger(() -> joystick.getRawButton(1));
    }

    public Trigger cruiseTrigger(){
        return new Trigger(() -> joystick.getRawButton(2));
    }
    public Trigger flashTrigger(){
        return new Trigger(() -> joystick.getRawButton(3));
    }
    public Trigger audioTrigger(){
        return new Trigger(() -> joystick.getRawButton(4));
    }
    public Trigger wipeTrigger(){
        return new Trigger(() -> joystick.getRawButton(5) );
    }
    public Trigger mapTrigger(){
        return new Trigger(() -> joystick.getRawButton(6));
    }
    public Trigger lightonTrigger(){
        return new Trigger(() -> joystick.getRawButton(7));
    }
    public Trigger talkonTrigger(){
        return new Trigger(() -> joystick.getRawButton(8));
    }
        }
    
    
    
    


