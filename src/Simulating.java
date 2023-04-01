public class Simulating {
    private static final double I=0.0003;
    private static final double D=0.2;
    private static final double P=0.04;
    private static final double MAX_I=100;
    private static final double MAX_ALT=30000;

    private static final double MIN_ALT=2000;


    private double _P,_I,_D,_max_i,_max_alt,_min_alt;
    private Bereshit _ber;
    private PID _pid;

    public Simulating()
    {
        this(P,I,D,MAX_I,MAX_ALT,MIN_ALT,new Bereshit());
    }
    public Simulating(double _P, double _I, double _D, double _max_i, double _max_alt, double _min_alt, Bereshit _ber) {
        this._P = _P;
        this._I = _I;
        this._D = _D;
        this._max_i = _max_i;
        this._max_alt = _max_alt;
        this._min_alt = _min_alt;
        this._ber = _ber;
        this._pid=new PID(_P,_I,_D,_max_i);
        this.start();
    }

    private void start()
    {
        System.out.println("Simulating Bereshit's Landing:");

        System.out.println("time \t vs \t hs \t fuel \t ang \t power \t alt \t dhs \tdvs \tPID");

        double hs=_ber.get_hs(),vs=_ber.get_vs(),dhs,dvs,error,gas,angC;
        double digits=digits((int)hs);
        digits=Math.pow(10,digits);
        double alpha_hs=digits*10;//10000;


        PIDController HS_PID = new PIDController(hs/alpha_hs, hs/alpha_hs, hs/alpha_hs);
        HS_PID.setOutputLimits(10);
        HS_PID.setSetpointRange(40);
        double HS_target=1;
        double HS_output=0;
        HS_PID.setSetpoint(0);
        HS_PID.setSetpoint(HS_target);


        PIDController VS_miniPID = new PIDController(vs/alpha_hs, vs/alpha_hs, vs/alpha_hs);
        VS_miniPID.setOutputLimits(1);
        VS_miniPID.setSetpointRange(40);
        double VS_target=1;
        double VS_output=0;
        VS_miniPID.setSetpoint(0);
        VS_miniPID.setSetpoint(VS_target);


        while (_ber.get_alt()>0)
        {
            HS_output = HS_PID.getOutput(_ber.get_hs(), HS_target);
            _ber.set_hs(HS_output+_ber.get_hs());;

            VS_output = VS_miniPID.getOutput(vs, VS_target);
            _ber.set_vs(vs+VS_output);


            _ber.step();
            hs=_ber.get_hs();
            vs=_ber.get_vs();
            //over 2 km
            dhs=desired_hs(_ber.get_alt());
            dvs=desired_vs(_ber.get_alt());
            error=vs-dvs;
            gas=_pid.update(error,_ber.get_dt());
            _ber.addPower(gas);
            if(_ber.get_time()%10==0 || _ber.get_dt()<100)
            {
                System.out.println(_ber+"\t"+dhs+" \t"+dvs+" \t"+gas);
            }
            if(_ber.get_ang()>3) {_ber.set_ang(_ber.get_ang()-3);} // rotate to vertical position.
            else {_ber.set_ang(0);}
            if(Math.abs(hs)<3)
            {
                angC=_ber.get_ang();
                if(angC>=3*_ber.get_dt())
                {
                    _ber.addAng(-3*_ber.get_dt());
                }
                else
                {
                    _ber.addAng(-angC);
                }
            }
        }

    }


    public  static  double desired_vs(double alt)
    {
        if(alt>MAX_ALT)
        {
            return 0;
        }
        if(alt>500){return 24;}
        if(alt>200){return 12;}
        if(alt>50){return 6;}
        if(alt>20){return 2;}
        return 1;

    }

    public  static  double desired_hs(double alt)
    {
        if(alt<MIN_ALT)
        {
            return 0;
        }
        if(alt>MAX_ALT)
        {
            return Moon.EQ_SPEED;
        }
        double norm=(alt-MIN_ALT)/(MAX_ALT-MIN_ALT);//[0,1]
        norm=Math.pow(norm,0.70);//[0,1]
        double dhs=norm*Moon.EQ_SPEED;
        return dhs;

    }


    public double get_P() {
        return _P;
    }

    public void set_P(double _P) {
        this._P = _P;
    }

    public double get_I() {
        return _I;
    }

    public void set_I(double _I) {
        this._I = _I;
    }

    public double get_D() {
        return _D;
    }

    public void set_D(double _D) {
        this._D = _D;
    }

    public double get_max_i() {
        return _max_i;
    }

    public void set_max_i(double _max_i) {
        this._max_i = _max_i;
    }

    public double get_max_alt() {
        return _max_alt;
    }

    public void set_max_alt(double _max_alt) {
        this._max_alt = _max_alt;
    }

    public double get_min_alt() {
        return _min_alt;
    }

    public void set_min_alt(double _min_alt) {
        this._min_alt = _min_alt;
    }

    public Bereshit get_ber() {
        return _ber;
    }

    public void set_ber(Bereshit _ber) {
        this._ber = _ber;
    }

    public PID get_pid() {
        return _pid;
    }

    public void set_pid(PID _pid) {
        this._pid = _pid;
    }

    private static int digits( int num)
    {
        int digits = 0;
        while(num != 0)
        {
            num/=10;
            digits++;
        }
        return digits;
    }
}
