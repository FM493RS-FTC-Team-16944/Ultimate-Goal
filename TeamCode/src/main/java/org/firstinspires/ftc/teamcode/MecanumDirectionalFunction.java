package org.firstinspires.ftc.teamcode;


/*
This algorithm is used to simplify programming for mecanum drivetrain
by taking as input the x and y directions and any rotational motion,
and returning the motor powers. Usage:

    MecanumDirectionalFunction m = new MecanumDirectionalFunction();             //Instantiate object m of class MecanumDirectionalFunction *Object does not have to be called m*
    m.Calculation(SidewaysPower, UpDownPower, RotationalPower);                  //Calculate
    FrontLeftDrive.setPower(m.GetFrontLeftPower());                              //Set Motor powers
    BackLeftDrive.setPower(m.GetBackLeftPower());
    FrontRightDrive.setPower(m.GetFrontRightPower());
    BackRightDrive.setPower(m.GetBackRightPower());


 */
public class MecanumDirectionalFunction{

private double[] powers = new double[4];

    public void Calculation(double xPower, double yPower, double rotPower)  {

        powers[0] = (yPower - xPower - rotPower);           //Front Left Motor Power
        powers[1] = (- yPower - xPower + rotPower);         //Back Left Motor power
        powers[2] = (yPower + xPower + rotPower);           //Front Right Motor Power
        powers[3] = (yPower - xPower + rotPower);           //Back Right Motor Power


        //Scale all motor powers to ensure that movement and directionality are all maintained
        if (Math.abs(powers[0]) > 1 || Math.abs(powers[1]) > 1 ||
                Math.abs(powers[2]) > 1 || Math.abs(powers[3]) > 1 ) {
            // Find the largest power

            double max = 0;
            max = Math.max(Math.abs(powers[0]), Math.abs(powers[1]));
            max = Math.max(Math.abs(powers[2]), max);
            max = Math.max(Math.abs(powers[3]), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs)
            powers[0] /= max;
            powers[1] /= max;
            powers[2] /= max;
            powers[3] /= max;

        }

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
