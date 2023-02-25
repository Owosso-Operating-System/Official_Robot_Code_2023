// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */

/** Class: PIDMath
   * Creates PID command.
   * The class calculates the direction needed to face foreward.
   *  */

public class PIDTurn {
   
    //creates the kP, kI and, kD variables and assigns their numerical values
    static double kP = 0.00375;
    static double kI = 0.0000026345;
    static double kD = 0.0004965;
    
    //creates the proportional, integral and, derivative variables
    static double proportional;
    static double integral;
    static double derivative;
    
    //creates the kAngleSetpoint and SpeedLimit variables
    static double kAngleSetpoint;
    static double speedLimit = 0.10;
    
    //creates the error, totalError, and lastError variables and sets their numerical values to 0
    static double error = 0;
    static double totalError = 0;
    static double lastError = 0;

    //creates the speed variable and sets its numerical value to 0
    static double speed = 0;

    public static double getSpeed(DriveTrain driveTrain, int setAngle){
    
        //Takes all of the preveous variables and takes the gyro values and puts them through this function 
        // allow the robot to smoothly travel to a specific angle

        kAngleSetpoint = setAngle;
        
        lastError = error;
        error = kAngleSetpoint - DriveTrain.gyro.getYaw();
        totalError += error;
    
        proportional = error * kP;
        integral = totalError * kI;
        derivative = (error - lastError) * kD;
    
        double output = proportional + integral + derivative;
    
        speedLimit = Math.copySign(speedLimit, output);
    
        if(output > 0){
            speed = output > speedLimit ? speedLimit : output;
        }
        else{
            speed = output < speedLimit ? speedLimit : output;
        }
        return speed;
        }
}