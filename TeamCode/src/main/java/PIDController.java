import static java.lang.Math.max;
import static java.lang.Math.min;

public class PIDController {
    /** Creates a configurable PID Controller
     *
     * kp = Proportional Tuning Constant
     * ti = Integral Tuning Constant
     * td = Derivative tuning Constant
     *
     * Define bounds of integral by integralMin, integralMax
     *
     * Define bounds of output by outMin, outMax
     */

    // Tuning Constants
    private double kp;
    private double ti;
    private double td;

    // Integral Max/Min Clamps
    private double integralMin;
    private double integralMax;

    // Output Max/Min Clamps
    private double outMin;
    private double outMax;

    //Errors
    private double nowE = 0;
    private double prevE = 0;
    private double runningE = 0;

    //Constructor
    public PIDController(double kp, double ti, double td, double integralMin, double integralMax, double outMin, double outMax){
        this.kp = kp;
        this.ti = ti;
        this.td = td;
        this.integralMin = integralMin;
        this.integralMax = integralMax;
        this.outMin = outMin;
        this.outMax = outMax;
    }

    // Clamps values between min & max restrictions
    private double clamp( double min, double max, double input){
        return min( max(min,input), max);
    }

    // Actual PID calculation
    public double getOutput(double processValue, double setPoint, double dT){
        /** Returns PID Control Output
         *  processValue (PV) = measured (controlled) variable
         *  setPoint (SP) = Value to which PV is attempting to head to.
         *
         */

        this.nowE = setPoint - processValue;
        runningE = clamp(this.integralMin,integralMax,runningE + (nowE * dT));
        double slopeE = (nowE - prevE)/dT;

        double output = this.kp * (nowE + runningE /ti) + (td * slopeE);

        prevE = nowE;
        return clamp(outMin,outMax,output);
    }

}
