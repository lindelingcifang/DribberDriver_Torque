#ifndef __UTILS_HPP
#define __UTILS_HPP

#include <cmath>

#define PI (3.1415926f)

template <typename T>
T limit(T data_t, T limit_t)
{
    if (limit_t < 0)
    {
        limit_t = -limit_t;
    }

    if (data_t > limit_t)
    {
        return limit_t;
    }
    else if (data_t < -limit_t)
    {
        return -limit_t;
    }
    else
    {
        return data_t;
    }
}

template <typename T>
T normalize(T data_t, T cycle_low, T cycle_high)
{
    if (cycle_low > cycle_high)
    {
        float t = cycle_low;
        cycle_low = cycle_high;
        cycle_high = t;
    }

    float cycle = cycle_high - cycle_low;

    while (data_t > cycle_high)
    {
        data_t -= cycle;
    }

    while (data_t < cycle_low)
    {
        data_t += cycle;
    }

    return data_t;
}

class PID {
public:
    struct Parameter_t
    {
        float kp;
        float ki;
        float kd;
        float output_limit;
        float integ_limit;
        float dt;
    };

    PID(const Parameter_t parameter) : parameter_(parameter) {};
    ~PID() = default;

    float calc(float ref, float fdb) {
        last_err = err;
        err = ref - fdb;

        diff = (err - last_err) / parameter_.dt;
        integ = limit<float>(integ + err * parameter_.dt, parameter_.integ_limit / parameter_.ki);

        output = limit<float>(parameter_.kp * err + parameter_.ki * integ + parameter_.kd * diff,
                              parameter_.output_limit);

        return output;
    }
    void set_parameter(const Parameter_t parameter) {
        parameter_ = parameter;
    }
    void reset() {
        integ = 0.0f;
        diff = 0.0f;
        last_err = 0.0f;
        output = 0.0f;
    }

private:
    Parameter_t parameter_;
    float err = 0.0f;
    float last_err = 0.0f;
    float integ = 0.0f;
    float diff = 0.0f;
    float output = 0.0f;    
};

class TD {
public:
    struct Parameter_t
    {
        float r;
        float h;
        float dt;
        bool is_cycle;
        float cycle_low;
        float cycle_high;
    };

    TD(Parameter_t parameter, float init) : parameter_(parameter) {
        x1 = init;
        x1k = init;
        x2 = 0;
        x2k = 0;
        d = parameter_.r * parameter_.h;
        d0 = d * parameter_.h;
        cycle = parameter_.cycle_high - parameter_.cycle_low;
    }
    ~TD() = default;
    void calc(float raw_data) {
        if (parameter_.is_cycle) {
            float delta_raw_data = raw_data - last_raw_data_;
            if (delta_raw_data > cycle / 2.0) {
                raw_data_ += delta_raw_data - cycle;
            }
            else if (delta_raw_data < -cycle / 2.0){
                raw_data_ += delta_raw_data + cycle;
            }
            else {
                raw_data_ += delta_raw_data;
            }
        }
        else {
            raw_data_ = raw_data;
        }

        x1k = x1;
        x2k = x2;

        float tdy = x1k - raw_data_ + parameter_.h * x2k;
        float a0 = sqrt(d * d + 8 * parameter_.r * abs(tdy));

        float a = 0;

        if (abs(tdy) <= d0) {
            a = x2k + tdy / parameter_.h;
        }
        else {
            if (tdy > 0) {
                a = x2k + 0.5 * (a0 - d);
            }
            else {
                a = x2k - 0.5 * (a0 - d);
            }
        }

        float f = 0;
        if (abs(a) <= d) {
            f = -parameter_.r * a / d;
        }
        else {
            if (a > 0) {
                f = -parameter_.r;
            }
            else {
                f = parameter_.r;
            }
        }

        x1 = x1k + parameter_.dt * x2k;
        x2 = x2k + parameter_.dt * f;

        if (parameter_.is_cycle) {
            data = normalize<float>(x1, parameter_.cycle_low, parameter_.cycle_high);
        }
        else {
            data = x1;
        }

        diff = x2;
        last_raw_data_ = raw_data;
    }
    float get_data() const { return data; };
    float get_diff() const { return diff; };

private:
    Parameter_t parameter_;
    float x1;
    float x2;
    float x1k;
    float x2k;
    float d;
    float d0;
    float data;
    float diff;
    float last_raw_data_;
    float raw_data_;
    float cycle;
};

#endif // __UTILS_HPP