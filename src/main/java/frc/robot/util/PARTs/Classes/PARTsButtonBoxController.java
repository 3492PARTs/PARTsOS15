// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.PARTs.Classes;

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
    public Trigger escTrigger(){
        return new Trigger(() -> joystick.getRawButton(9));
    }
    public Trigger enterTrigger(){
        return new Trigger(() -> joystick.getRawButton(10));
    }
    public Trigger enginestartTrigger(){
        return new Trigger(() -> joystick.getRawButton(11));
    }
    public Trigger nukeTrigger(){
        return new Trigger(() -> joystick.getRawButton(12));
    }
    public Trigger positive1Trigger(){
        return new Trigger(() -> joystick.getRawButton(13));
    }
    public Trigger negative1Trigger(){
        return new Trigger(() -> joystick.getRawButton(14));
    }
    public Trigger positive2Trigger(){
        return new Trigger(() -> joystick.getRawButton(15));

    }
    public Trigger negative2Trigger(){
        return new Trigger(() -> joystick.getRawButton(16));
    }
    public Trigger positive3Trigger(){
        return new Trigger(() -> joystick.getRawButton(17));
    }
    public Trigger negative3Trigger(){
        return new Trigger(() -> joystick.getRawButton(18));
    }
    public Trigger positive4Trigger(){
        return new Trigger(() -> joystick.getRawButton(19));
    }
    public Trigger negative4Trigger(){
        return new Trigger(() -> joystick.getRawButton(20));
    }
    public Trigger absrightTrigger(){
        return new Trigger(() -> joystick.getRawButton(21));
    }
    public Trigger absleftTrigger(){
        return new Trigger (() -> joystick.getRawButton(22));
    }
    public Trigger tcrightTrigger(){
        return new Trigger (() -> joystick.getRawButton(23));
    }
    public Trigger tcleftTrigger(){
        return new Trigger(() -> joystick.getRawButton(24));
    }
    public Trigger absclickTrigger(){
        return new Trigger(() -> joystick.getRawButton(25));
    }
    public Trigger tcclickTrigger(){
        return new Trigger(() -> joystick.getRawButton(26));
    }
    public Trigger povTrigger1(){
        return new Trigger(()  -> joystick.getPOV(0)== -1);  //this one is negitive 
    }
    public Trigger povTrigger0(){
        return new Trigger(()  -> joystick.getPOV(0)== 0);
    }
    public Trigger povTrigger45(){
        return new Trigger(()  -> joystick.getPOV(0)== 45);
    }
    public Trigger povTrigger90(){
        return new Trigger(()  -> joystick.getPOV(0)== 90);
    }
    public Trigger povTrigger135(){
        return new Trigger(()  -> joystick.getPOV(0)==135);
    }
    public Trigger povTrigger180(){
        return new Trigger(()  -> joystick.getPOV(0)==180);
    }
    public Trigger povTrigger225(){
        return new Trigger(()  -> joystick.getPOV(0)==225);
    }
    public Trigger povTrigger270(){
        return new Trigger(()  -> joystick.getPOV(0)==270);
    }
    public Trigger povTrigger315(){
        return new Trigger(()  -> joystick.getPOV(0)==315);
    }
    }
    
    
    
    
    
    


