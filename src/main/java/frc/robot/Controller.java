package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;

public class Controller {
    private final GenericHID controller;
    private double[] errors;

    public Controller(int id){
        controller = new GenericHID(id);
        for(int i=0; i<errors.length; i++){
            errors[i] = controller.getRawAxis(i);
        }
    }

    public double getAxis(int axis){
        if(axis%2 == 0){
            double x = getRawAxis(axis);
            double y = getRawAxis(axis+1);
            return x *Math.sqrt(1 - 0.5*y*y);
        }else{
            double x = getRawAxis(axis-1);
            double y = getRawAxis(axis);
            return y *Math.sqrt(1 - 0.5*x*x);
        }
    }

    public double getRawAxis(int axis){
        double value = controller.getRawAxis(axis)-errors[axis];
        double cap = Math.min(1+errors[axis], 1-errors[axis]);
        if(value>cap){
            value = cap;
        }else if(value<-cap){
            value = -cap;
        }
        value/=cap;

        return value;
    }
}
