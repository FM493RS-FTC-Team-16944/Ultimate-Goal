package org.firstinspires.ftc.teamcode;

public class AutoAlign {

    private Double Distanc3, Di2tance, Angle;
    private double[] powers = new double[4];


    public void Math(double Xtarget, double Ytarget, double Ztarget, double Xpos, Double Ypos, Double Zpos){
        Distanc3 = Math.sqrt((Math.pow((Xtarget-Xpos),2)) + (Math.pow((Ytarget-Ypos),2)) + (Math.pow((Ztarget-Zpos),2)));             //Calculates distance from the robot to a target point in 3D space
        Di2tance = Math.sqrt((Math.pow((Xtarget-Xpos),2)) + (Math.pow((Ytarget-Ypos),2)));                                            //Calculates distance from the robot to a target point on the ground plane
        Angle = Math.toDegrees(Math.atan((Ytarget-Ypos)/(Xtarget-Xpos)));                                                             //Calculates the apropriate angle of position for the robot

    }
    public void Orient(double Yaw) {
        MecanumDirectionalFunction O = new MecanumDirectionalFunction();
        if(Yaw < Angle){
            O.Calculation(0,0,-0.3);
        } else if (Yaw > Angle){
            O.Calculation(0,0,0.3);
        }

        powers[0] = O.GetFrontLeftPower();
        powers[1] = O.GetBackLeftPower();
        powers[2] = O.GetFrontRightPower();
        powers[3] = O.GetBackRightPower();

    }

    public double GetAngle(){
        return Angle;
    }
    public Double GetGroundDist(){
        return Di2tance;
    }

    public double GetFrontLeftPower(){
        return powers[0];                         //Return the Front left motor power as calculated
    }
    public double GetBackLeftPower(){
        return powers[1];                         //Return the Back left motor power as calculated
    }
    public double GetFrontRightPower(){
        return powers[2];                         //Return the Front right motor power as calculated
    }
    public double GetBackRightPower(){
        return powers[3];                         //Return the back right motor power as calculated
    }




}
