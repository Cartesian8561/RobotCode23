// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class CartesianEncoder {
    private double previousPosition = 300.0;
    private int rotationCount = 0;
    private double encoderConstant;
    private double distance = 0.0;

    private long currentTime;
    private long prevTime = 0L;
    private double velocity;
    private double offset = 0.0;

    public CartesianEncoder(double encoderConstant, double offset){
        this.encoderConstant = encoderConstant;
        this.offset = offset; 
    }

    public void resetRotation(double offset){
        rotationCount = 0;
        this.offset = offset;
    }
    
    public void setRotation(int rotation){
        rotationCount = rotation;
    }

    public void setValues(int rotation, double position, double currentPosition){
        rotationCount = rotation;
        offset = currentPosition - position;

    }

    public double getDistance(double position){
        distance = (position - offset) + rotationCount * encoderConstant;
        SmartDashboard.putNumber("rotation count", rotationCount);
        return distance;
    }

    /**voltage is negative when going forward */
    public void updateNegativeDistance(double position, double volts){
        currentTime = System.currentTimeMillis();
        if(prevTime == 0L){
            prevTime = currentTime;
        }
        if(currentTime - prevTime < 4){
            return;
        }
        
        if(previousPosition == 30.0){
            previousPosition = position;
        } 
        
        if(volts < 0){
            if(position < previousPosition){
                rotationCount++;
                distance = rotationCount * encoderConstant + position;
            }
        } else if(volts == 0){
            previousPosition = position;
            prevTime = currentTime;
            return;
        } else{
            if(position > previousPosition){
                rotationCount--;
                distance = rotationCount * encoderConstant + position;
            }
        }
        previousPosition = position;
        prevTime = currentTime;
    }


    /**voltage is positive when going forward*/
    public void updatePositiveDistance(double position, double volts){
        currentTime = System.currentTimeMillis();
        if(prevTime == 0L){
            prevTime = currentTime;
        }
        if(currentTime - prevTime < 4 || position == previousPosition){
            return;
        }
        
        if(previousPosition == 30.0){
            previousPosition = position;
        } 
        
        if(volts > 0){
            if(position < previousPosition){
                rotationCount++;
                distance = rotationCount * encoderConstant + position;
            }
        } else if(volts == 0){
            previousPosition = position;
            prevTime = currentTime;
            return;
        } else{
            if(position > previousPosition){
                rotationCount--;
                distance = rotationCount * encoderConstant + position;
            }
        }
        previousPosition = position;
        prevTime = currentTime;
    }




    public void updateDistance(double rawPosition){
        double position = (rawPosition - offset);
        if(previousPosition == 300.0){
            previousPosition = position;
        }
        
        velocity = (previousPosition - position) / 4.0;
        SmartDashboard.putNumber("previous position", previousPosition);
        SmartDashboard.putNumber("current position", position);
        SmartDashboard.putNumber("arm velocity", velocity); 
        currentTime = System.currentTimeMillis();


        if(prevTime == 0L){
            prevTime = currentTime;
        }

        if(currentTime - prevTime < 4 || position == previousPosition){
            return;
        }        

  
        if (Math.abs(velocity) < 0.001){ //buradaki değişkeni velocity'nin 0 değerine göre belirle
            previousPosition = position;
            prevTime = currentTime;
            return;
        }

        if(velocity > 0.12){
            rotationCount++;
            distance = rotationCount * encoderConstant + position;
        }else if(velocity < -0.12){
            rotationCount--;
            distance = rotationCount * encoderConstant + position;
        }

        previousPosition = position;
        prevTime = currentTime;
    }
}
