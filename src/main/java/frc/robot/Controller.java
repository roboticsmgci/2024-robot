package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Controller {
    public final CommandGenericHID controller;
    public double[] errors = new double[6];
    private double sectorAngle = 0.28255495247;
    private double maxRadius = 1.408973;

    public Controller(CommandGenericHID controller){
        this.controller = controller;
        // for(int i=0; i<errors.length; i++){
        //     errors[i] = controller.getRawAxis(i);
        // }
    }

    public CommandGenericHID getController(){
        return controller;
    }

    public Trigger getButton(int button){
        return controller.button(button);
    }

    // Map the input of the joystick to a circle
    public double getAxis(int axis){
        double x = Math.abs(getRawAxis(axis-axis%2));
        double y = Math.abs(getRawAxis(axis-axis%2+1));
        double angle = Math.atan2(y, x);
        double magnitude = 1;

        if(angle < Math.PI/4-sectorAngle){
            magnitude = Math.sqrt(1+Math.sin(angle));
        }else if(angle <= Math.PI/4+sectorAngle){
            // Circle section
            magnitude = maxRadius;
        }else{
            magnitude = Math.sqrt(1+Math.cos(angle));
        }
        return getRawAxis(axis)/magnitude;
    }

    // Cancel out the error (drift)
    public double getRawAxis(int axis){
        double value = controller.getRawAxis(axis)-errors[axis];
        double cap = Math.min(1+errors[axis], 1-errors[axis]);
        if(value>cap){
            value = cap;
        }else if(value<-cap){
            value = -cap;
        }
        value/=cap;

        if(axis%2==1){
            value*=-1;
        }

        return value;
    }
}
