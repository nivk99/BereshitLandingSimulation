
// Some of the code is from: https://github.com/tekdemo/MiniPID-Java



public class PIDController {

    private double P=0;
    private double I=0;
    private double D=0;
    private double F=0;

    private double maxIOutput=0;
    private double maxError=0;
    private double errorSum=0;

    private double maxOutput=0;
    private double minOutput=0;

    private double setpoint=0;

    private double lastActual=0;

    private boolean firstRun=true;
    private boolean reversed=false;

    private double outputRampRate=0;
    private double lastOutput=0;

    private double outputFilter=0;

    private double setpointRange=0;

    public PIDController(double p, double i, double d){
        P=p; I=i; D=d;
        checkSigns();
    }


    public PIDController(double p, double i, double d, double f){
        P=p; I=i; D=d; F=f;
        checkSigns();
    }


    public void setP(double p){
        P=p;
        checkSigns();
    }

    public void setI(double i)
    {
        if(I!=0){
            errorSum=errorSum*I/i;
        }
        if(maxIOutput!=0){
            maxError=maxIOutput/i;
        }
        I=i;
        checkSigns();
    }

    public void setD(double d)
    {
        D=d;
        checkSigns();
    }


    public double getP() {
        return P;
    }

    public double getI() {
        return I;
    }

    public double getD() {
        return D;
    }

    /**
     * Set the maximum output value contributed by the I component of the system
     * This can be used to prevent large windup issues and make tuning simpler
     * @param maximum. Units are the same as the expected output value
     */
    public void setMaxIOutput(double maximum){
        // Internally maxError and Izone are similar, but scaled for different purposes.
        // The maxError is generated for simplifying math, since calculations against
        // the max error are far more common than changing the I term or Izone.
        maxIOutput=maximum;
        if(I!=0){
            maxError=maxIOutput/I;
        }
    }

    /**
     * Specify a maximum output range. <br>
     * When one input is specified, output range is configured to
     * <b>[-output, output]</b>
     * @param output
     */
    public void setOutputLimits(double output){
        setOutputLimits(-output,output);
    }

    /**
     * Specify a  maximum output.
     * When two inputs specified, output range is configured to
     * <b>[minimum, maximum]</b>
     * @param minimum possible output value
     * @param maximum possible output value
     */
    public void setOutputLimits(double minimum,double maximum){
        if(maximum<minimum)return;
        maxOutput=maximum;
        minOutput=minimum;

        // Ensure the bounds of the I term are within the bounds of the allowable output swing
        if(maxIOutput==0 || maxIOutput>(maximum-minimum) ){
            setMaxIOutput(maximum-minimum);
        }
    }

    /**
     * Set the operating direction of the PID controller
     * @param reversed Set true to reverse PID output
     */
    public void  setDirection(boolean reversed){
        this.reversed=reversed;
    }

    //**********************************
    // Primary operating functions
    //**********************************

    /**
     * Configure setpoint for the PID calculations<br>
     * This represents the target for the PID system's, such as a
     * position, velocity, or angle. <br>
     * @see PIDController#getOutput(actual) <br>
     * @param setpoint
     */
    public void setSetpoint(double setpoint){
        this.setpoint=setpoint;
    }

    /**
     * Calculate the output value for the current PID cycle.<br>
     * @param actual The monitored value, typically as a sensor input.
     * @param setpoint The target value for the system
     * @return calculated output value for driving the system
     */
    public double getOutput(double actual, double setpoint){
        double output;
        double Poutput;
        double Ioutput;
        double Doutput;
        double Foutput;

        this.setpoint=setpoint;

        // Ramp the setpoint used for calculations if user has opted to do so
        if(setpointRange!=0){
            setpoint=constrain(setpoint,actual-setpointRange,actual+setpointRange);
        }

        // Do the simple parts of the calculations
        double error=setpoint-actual;

        // Calculate F output. Notice, this depends only on the setpoint, and not the error.
        Foutput=F*setpoint;

        // Calculate P term
        Poutput=P*error;

        // If this is our first time running this, we don't actually _have_ a previous input or output.
        // For sensor, sanely assume it was exactly where it is now.
        // For last output, we can assume it's the current time-independent outputs.
        if(firstRun){
            lastActual=actual;
            lastOutput=Poutput+Foutput;
            firstRun=false;
        }

        // Calculate D Term
        // Note, this is negative. This actually "slows" the system if it's doing
        // the correct thing, and small values helps prevent output spikes and overshoot
        Doutput= -D*(actual-lastActual);
        lastActual=actual;

        // The Iterm is more complex. There's several things to factor in to make it easier to deal with.
        // 1. maxIoutput restricts the amount of output contributed by the Iterm.
        // 2. prevent windup by not increasing errorSum if we're already running against our max Ioutput
        // 3. prevent windup by not increasing errorSum if output is output=maxOutput
        Ioutput=I*errorSum;
        if(maxIOutput!=0){
            Ioutput=constrain(Ioutput,-maxIOutput,maxIOutput);
        }

        // And, finally, we can just add the terms up
        output=Foutput + Poutput + Ioutput + Doutput;

        // Figure out what we're doing with the error.
        if(minOutput!=maxOutput && !bounded(output, minOutput,maxOutput) ){
            errorSum=error;
            // reset the error sum to a sane level
            // Setting to current error ensures a smooth transition when the P term
            // decreases enough for the I term to start acting upon the controller
            // From that point the I term will build up as would be expected
        }
        else if(outputRampRate!=0 && !bounded(output, lastOutput-outputRampRate,lastOutput+outputRampRate) ){
            errorSum=error;
        }
        else if(maxIOutput!=0){
            errorSum=constrain(errorSum+error,-maxError,maxError);
            // In addition to output limiting directly, we also want to prevent I term
            // buildup, so restrict the error directly
        }
        else{
            errorSum+=error;
        }

        // Restrict output to our specified output and ramp limits
        if(outputRampRate!=0){
            output=constrain(output, lastOutput-outputRampRate,lastOutput+outputRampRate);
        }
        if(minOutput!=maxOutput){
            output=constrain(output, minOutput,maxOutput);
        }
        if(outputFilter!=0){
            output=lastOutput*outputFilter+output*(1-outputFilter);
        }

        // Get a test printline with lots of details about the internal
        // calculations. This can be useful for debugging.
        // System.out.printf("Final output %5.2f [ %5.2f, %5.2f , %5.2f  ], eSum %.2f\n",output,Poutput, Ioutput, Doutput,errorSum );
        // System.out.printf("%5.2f\t%5.2f\t%5.2f\t%5.2f\n",output,Poutput, Ioutput, Doutput );

        lastOutput=output;
        return output;
    }


    public void setSetpointRange(double range){
        setpointRange=range;
    }


    //**************************************
    // Helper functions
    //**************************************

    /**
     * Forces a value into a specific range
     * @param value input value
     * @param min maximum returned value
     * @param max minimum value in range
     * @return Value if it's within provided range, min or max otherwise
     */
    private double constrain(double value, double min, double max){
        if(value > max){ return max;}
        if(value < min){ return min;}
        return value;
    }

    /**
     * Test if the value is within the min and max, inclusive
     * @param value to test
     * @param min Minimum value of range
     * @param max Maximum value of range
     * @return true if value is within range, false otherwise
     */
    private boolean bounded(double value, double min, double max){
        // Note, this is an inclusive range. This is so tests like
        // `bounded(constrain(0,0,1),0,1)` will return false.
        // This is more helpful for determining edge-case behaviour
        // than <= is.
        return (min<value) && (value<max);
    }

    /**
     * To operate correctly, all PID parameters require the same sign
     * This should align with the {@literal}reversed value
     */
    private void checkSigns(){
        if(reversed){  // all values should be below zero
            if(P>0) P*=-1;
            if(I>0) I*=-1;
            if(D>0) D*=-1;
            if(F>0) F*=-1;
        }
        else{  // all values should be above zero
            if(P<0) P*=-1;
            if(I<0) I*=-1;
            if(D<0) D*=-1;
            if(F<0) F*=-1;
        }
    }



    public static void main(String[] args) {
        PIDController miniPID;

        miniPID = new PIDController(0.25, 0.01, 0.4);
        miniPID.setOutputLimits(100);
        miniPID.setSetpointRange(40);

        double target=1;

        double actual=900;
        double output=0;

        miniPID.setSetpoint(0);
        miniPID.setSetpoint(target);

        System.err.printf("Target\tActual\tOutput\tError\n");
        //System.err.printf("Output\tP\tI\tD\n");

        // Position based test code
        for (int i = 0; i < 100; i++){

            //if(i==50)miniPID.setI(.05);

            if (i == 60)
                break;
                //target = 50;

            //if(i==75)target=(100);
            //if(i>50 && i%4==0)target=target+(Math.random()-.5)*50;

            output = miniPID.getOutput(actual, target);
            actual = actual + output;

            //System.out.println("==========================");
            //System.out.printf("Current: %3.2f , Actual: %3.2f, Error: %3.2f\n",actual, output, (target-actual));
            System.err.printf("%3.2f\t%3.2f\t%3.2f\t%3.2f\n", target, actual, output, (target-actual));

            //if(i>80 && i%5==0)actual+=(Math.random()-.5)*20;
        }
    }
}