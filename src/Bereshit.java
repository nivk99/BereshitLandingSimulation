public class Bereshit {

    //Setting constants
    private static final double WEIGHT_EMP = 165; // kg
    private static final double WEIGHT_FULE = 420; // kg

    private static final double WEIGHT_FULL = WEIGHT_EMP + WEIGHT_FULE; // kg

    private static final double MAIN_ENG_F = 430; // N

    private static final double SECOND_ENG_F = 25; // N
    private static final double MAIN_BURN = 0.15; //liter per sec, 12 liter per m'
    private static final double SECOND_BURN = 0.009; //liter per sec 0.6 liter per m'

    private static final double ALL_BURN = MAIN_BURN + 8*SECOND_BURN;

    private static final double VS=24.8;

    private static final double HS=932;

    private static final double DIST=181*1000;

    private static final double ANG=58.3;

    private static final double ALT=13748;

    private static final double TIME=0;

    private static final double DT=1;

    private static final double ACC=0;

    private static final double FUEL=121;

    private static final double WEIGHT=WEIGHT_EMP + FUEL;

    private static final double NN=0.7;


    private  double _vs,_hs,_dist,_ang,_alt,_time,_dt,_acc,_fuel,_weight,_NN;

    public Bereshit()
    {
        this(VS,HS,DIST,ANG,ALT,TIME,DT,ACC,FUEL,WEIGHT,NN);
    }
    public Bereshit(double _vs, double _hs, double _dist, double _ang, double _alt, double _time, double _dt, double _acc, double _fuel, double _weight, double _NN)
    {
        this._vs = _vs;//מהירות אנכית
        this._hs = _hs;// מהירות אופקית
        this._dist = _dist;// מרחק
        this._ang = _ang;// זווית החללית
        this._alt = _alt;
        this._time = _time;// זמן שיגור
        this._dt = _dt;// קיבועי הזמן
        this._acc = _acc;// האצה
        this._fuel = _fuel;// דלק
        this._weight = _weight;//משקל
        this._NN = _NN;//עוצמת הגז
        start();
    }



    //get and set
    public double get_vs() {
        return _vs;
    }

    public void set_vs(double _vs) {
        this._vs = _vs;
    }

    public double get_hs() {
        return _hs;
    }

    public void set_hs(double _hs) {
        this._hs = _hs;
    }

    public double get_dist() {
        return _dist;
    }

    public void set_dist(double _dist) {
        this._dist = _dist;
    }

    public double get_ang() {
        return _ang;
    }

    public void set_ang(double _ang) {
        this._ang = _ang;
    }

    public double get_alt() {
        return _alt;
    }

    public void set_alt(double _alt) {
        this._alt = _alt;
    }

    public double get_time() {
        return _time;
    }

    public void set_time(double _time) {
        this._time = _time;
    }

    public double get_dt() {
        return _dt;
    }

    public void set_dt(double _dt) {
        this._dt = _dt;
    }

    public double get_acc() {
        return _acc;
    }

    public void set_acc(double _acc) {
        this._acc = _acc;
    }

    public double get_fuel() {
        return _fuel;
    }

    public void set_fuel(double _fuel) {
        this._fuel = _fuel;
    }

    public double get_weight() {
        return _weight;
    }

    public void set_weight(double _weight) {
        this._weight = _weight;
    }

    public double get_NN() {
        return _NN;
    }

    public void set_NN(double _NN) {
        this._NN = _NN;
    }
    //


    public  double accMax(double weight) {
        return acc(weight, true,8);
    }
    public  double acc(double weight, boolean main, int seconds) {
        double t = 0;
        if(main) {t += MAIN_ENG_F;}
        t += seconds*SECOND_ENG_F;
        double ans = t/weight;
        return ans;
    }

    public void  update()
    {
        double ang_rad = Math.toRadians(_ang);
        double h_acc = Math.sin(ang_rad)*_acc;
        double v_acc = Math.cos(ang_rad)*_acc;
        double vacc = Moon.getAcc(_hs);
        _time+=_dt;
        double as = (_hs/_alt); // Angular speed per second.
          _ang += as;
        if (_ang<0)
            _ang =1;
        double dw = _dt*ALL_BURN*_NN;
        if(_fuel>0) {
            _fuel -= dw;
            _weight = WEIGHT_EMP + _fuel;
            _acc = _NN* accMax(_weight);
        }
        else { // ran out of fuel
            _acc=0;
        }

        v_acc -= vacc; // moon affects the vertical acceleration
        if(_hs>0) {_hs -= h_acc*_dt;} // updating hs
        _dist -= _hs*_dt; // updating distance
        _vs -= v_acc*_dt;
        if(_vs < 0) { _vs = 1; }
        _alt -= _dt*_vs;  // updating alltitude
            if((_time % 10 == 0 || _alt<100)&&_alt>0)
            {
                print();
            }
    }
    private void start()
    {
        PIDController ANG_PID = new PIDController(0.25, 0.01, 0.4);
        ANG_PID.setOutputLimits(1);
        ANG_PID.setSetpointRange(40);
        double ANG_target=50;
        double ANG_output=0;
        ANG_PID.setSetpoint(0);
        ANG_PID.setSetpoint(ANG_target);




        PIDController HS_PID = new PIDController(_hs/10000, _hs/10000, _hs/10000);
        HS_PID.setOutputLimits(1);
        HS_PID.setSetpointRange(40);
        double HS_target=0;
        double HS_output=0;
        HS_PID.setSetpoint(0);
        HS_PID.setSetpoint(HS_target);


        PIDController VS_miniPID = new PIDController(_vs/1000, _vs/10000, _vs/1000);
        VS_miniPID.setOutputLimits(1);
        VS_miniPID.setSetpointRange(40);
        double VS_target=0;
        double VS_output=0;
        VS_miniPID.setSetpoint(0);
        VS_miniPID.setSetpoint(VS_target);




        while(_alt>0)
        {

            HS_output = HS_PID.getOutput(_hs, HS_target);
            _hs += HS_output;

            VS_output = VS_miniPID.getOutput(_vs, VS_target);
            _vs +=VS_output;

            if(_alt>2000)
            {
                if(_vs >25) {_NN+=0.003*_dt;} // more power for braking
                if(_vs <20) {_NN-=0.003*_dt;} // less power for braking

            }
            else
            {
                if(_ang>3) {_ang-=3;} // rotate to vertical position.
                else {_ang =0;}
                if(_ang > 20) {
                    _NN = 0.8;
                }
                else {
                    _NN = 0.9;
                }
                if(_hs<2) {_hs=0;}
                if(_alt<125)
                { // very close to the ground!
                    if(_vs<5) {_NN=0.7;}
                    else { _NN = 1; }// if it is slow enough - go easy on the brakes
                }
            }
            update();

        }

    }

    private void print()
    {
        if(_alt<10000)
        {
            System.err.printf("Time:%3.2f\t\tVS:%3.2f\t\tHS:%3.2f\t\tDIST:%3.2f\t\tALT:%3.2f\t\t\tANG:%3.2f\t\tWeight:%3.2f\t\tAcc:%3.2f\t\tFuel:%3.2f\n", _time, _vs, _hs, _dist, _alt, _ang, _weight, _acc, _fuel);

        }
        else
        {
            System.err.printf("Time:%3.2f\t\tVS:%3.2f\t\tHS:%3.2f\t\tDIST:%3.2f\t\tALT:%3.2f\t\tANG:%3.2f\t\tWeight:%3.2f\t\tAcc:%3.2f\t\tFuel:%3.2f\n", _time, _vs, _hs, _dist, _alt, _ang, _weight, _acc, _fuel);

        }
    }


}
