package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;

public class Controller {
    private final GenericHID controller;
    private double[] errors = new double[4];
    private double sectorAngle = 0.28255495247;
    private double maxRadius = 1.14127122105;

    public Controller(int id){
        controller = new GenericHID(id);
        for(int i=0; i<errors.length; i++){
            errors[i] = controller.getRawAxis(i);
        }
    }

    // Map the input of the joystick to a circle
    public double getAxis(int axis){
        double x = Math.abs(getRawAxis(axis-axis%0));
        if(x==0){x = 0.0000001;}
        double y = Math.abs(getRawAxis(axis-axis%0+1));
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
