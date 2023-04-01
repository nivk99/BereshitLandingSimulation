public class Bereshit {
    /**
     * This class represents the basic flight controller of the Bereshit space craft.
     *
     * @author ben-moshe
     */
    public static final double WEIGHT_EMP = 165; // kg
    public static final double WEIGHT_FULE = 420; // kg
    public static final double WEIGHT_FULL = WEIGHT_EMP + WEIGHT_FULE; // kg
    // https://davidson.weizmann.ac.il/online/askexpert/%D7%90%D7%99%D7%9A-%D7%9E%D7%98%D7%99%D7%A1%D7%99%D7%9D-%D7%97%D7%9C%D7%9C%D7%99%D7%AA-%D7%9C%D7%99%D7%A8%D7%97
    public static final double MAIN_ENG_F = 430; // N
    public static final double SECOND_ENG_F = 25; // N
    public static final double MAIN_BURN = 0.15; //liter per sec, 12 liter per m'
    public static final double SECOND_BURN = 0.009; //liter per sec 0.6 liter per m'
    public static final double ALL_BURN = MAIN_BURN + 8 * SECOND_BURN;






    private static final double VS=24.8;

    private static final double HS=932;

    private static final double DIST=181*1000;

    private static final double ANG=60;

    private static final double ALT=13748;

    private static final double TIME=0;

    private static final double DT=1;

    private static final double ACC=0;

    private static final double FUEL=121;

    private static final double NN=0.7;




    private double _power, _ang,_alt,_h_dist,_vs,_hs,_acc,_fuel,_weight,_time,_dt;

    public static double accMax(double weight) {
        return acc(weight, true, 8);
    }

    public static double acc(double weight, boolean main, int seconds) {
        double t = 0;
        if (main) {
            t += MAIN_ENG_F;
        }
        t += seconds * SECOND_ENG_F;
        double ans = t / weight;
        return ans;
    }


    public Bereshit(double power, double ang, double alt, double vs, double hs, double fuel, double dt, double h_dist) {
        this._power = power;
        this._ang = ang;
        this._alt = alt;
        this._vs = vs;
        this._hs = hs;
        this._fuel = fuel;
        this._dt=dt;
        this._h_dist=h_dist;
        this._acc = ACC;
        this._time = TIME;
        this._weight = fuel + WEIGHT_EMP;
    }

    public Bereshit() {
        this(NN,ANG,ALT,VS,HS,FUEL,DT,DIST);

    }

    @Override
    public String toString() {
        String ans = "" + _time + " \t" + (_vs) + " \t" + (_hs) + " \t" + (_fuel) + " \t" + (_ang)+ " \t" + (_power)+ " \t" + (_alt);
        return ans;

    }


    public void addPower(double d) {
        double dd = _power + d;
        if (dd >= 0.0 && dd <= 1) {
            _power = dd;
        }
    }

    public double get_power() {
        return _power;
    }

    public void set_power(double _power) {
        if (_power >= 0 && _power <= 1) {
            this._power = _power;
        }

    }

    public void addAng(double d) {
        double dd = _ang + d;
        if (dd >= 0 && dd <= 90) {
            _ang = dd;
        }
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

    public double get_h_dist() {
        return _h_dist;
    }

    public void set_h_dist(double _h_dist) {
        this._h_dist = _h_dist;
    }

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

    public void step()
    {
        double ang_rad = Math.toRadians(_ang);
        double h_acc = Math.sin(ang_rad) * _acc;
        double v_acc = Math.cos(ang_rad) * _acc;
        double vacc = Moon.getAcc(_hs);

        double as = (_hs/_alt); // Angular speed per second.
        _ang += as;

        _time += _dt;
        double dw = _dt * ALL_BURN * _power;
        if (_fuel > 0) {
            _fuel -= dw;
            _weight = WEIGHT_EMP + _fuel;
            _acc = _power * accMax(_weight);
        } else { // ran out of fuel
            _acc = 0;
        }

        v_acc -= vacc;
        if (_hs > 0) {
            _hs -= h_acc * _dt;
        }
        _h_dist -= _hs * _dt;
        _vs -= v_acc * _dt;
        _alt -= _dt * _vs;

        if(_hs<0)
            _hs=0;
        if(_vs<0)
            _vs=0;
    }


}





