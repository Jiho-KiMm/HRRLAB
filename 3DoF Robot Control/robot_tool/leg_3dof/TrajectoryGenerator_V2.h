/*
 * TrajectoryGenerator.h
 *
 *  Created on: 2021. 12. 23.
 *      Author: KimKyungHwan , HRRLAB , SeoulTech
 * 
 *  unit: meter[m], second[s]
 * 
 * ==========================================
 * 
        trajectory single-axis example
  
  [m]   3-	        _          _  
        2-	     _/  \_      /  \
        1-	___/       \____/    \  
        0-	                      \_______
   time[s]: 0 1 2 3 4 5 6 7 8 9 10 11 12...   
   state:   0  -1  1 -1 2   -1 3 -1 4 4 4....
  
#include <iostream>
#include "TrajectoryGenerator.h"
int main(int, char **)
{
    TrajectoryGenerator trajectory;
    trajectory.SetTimeInterval(0.01);
    int iter = 0;
    while (iter <= 1200)
    {
        if (trajectory.GetState() == 0)
        {
            trajectory.SetInitialPosition(0.5);
            trajectory.SetTrajectoryType(TrajectoryGenerator::SINUSOIDAL);
            trajectory.SetFinalPosition(3);
            trajectory.SetDuration(4);
        }
        else if (trajectory.GetState() == 1)
        {
            trajectory.SetTrajectoryType(TrajectoryGenerator::LSPB);
            trajectory.SetFinalPosition(1);
            trajectory.SetDuration(2, 1);
            trajectory.SetConstantVelocity(1.5);
        }
        else if (trajectory.GetState() == 2)
        {
            trajectory.SetTrajectoryType(TrajectoryGenerator::QUINTIC);
            trajectory.SetFinalPosition(3);
            trajectory.SetFinalVelocity(1);
            trajectory.SetFinalAcceleration(0.5);
            trajectory.SetDuration(2);
        }
        else if (trajectory.GetState() == 3)
        {
            trajectory.SetTrajectoryType(TrajectoryGenerator::QUINTIC);
            trajectory.SetBasicParameters(0., 2);
        }

        trajectory.ComputeTrajectory();

        std::cout << iter << "    " << trajectory.GetPosition()<<"    " <<trajectory.GetVelocity()<<"    " <<trajectory.GetAcceleration() << std::endl;
        iter++;
    }
    return 0;
}
 * 
 * 
 * ============================================
 */
#ifndef TRAJECTORYGENERATOR_H_
#define TRAJECTORYGENERATOR_H_

#include <cmath>
#include <iostream>

class TrajectoryGenerator
{
public:
    enum TrajectoryType
    {
        SINUSOIDAL = 0,
        LSPB = 1,
        QUINTIC = 2,
        BEZIER = 3,
    };

private:
    enum TrajectoryMode
    {
        POINT_SINGLE = 1,
        POINT_XYZ = 3,
    };
    enum Axis
    {
        SINGLE_AXIS = 0,
        X = 0,
        Y = 1,
        Z = 2,
        NUM_AXIS = 3,
    };
    const double kEpsilon;
    const double kPi;
    double dt_; //delta time, default 0.001s
    double t_;  // trajectory time -> 0~duration
    double gt_; // global time -> 0~ Inf  !!Unimplemented!!
    unsigned int trajectory_type_;
    unsigned int trajectory_mode_;
    double pi_[NUM_AXIS];      //initial position
    double pf_[NUM_AXIS];      //final position
    double vi_[NUM_AXIS];      //initial velocity
    double vf_[NUM_AXIS];      //final velocity
    double ai_[NUM_AXIS];      //initial acceleration
    double af_[NUM_AXIS];      //final acceleration
    double tf_;                //final time(=duration)
    double v_const_[NUM_AXIS]; //LSPB constant velocity
    double tb_[NUM_AXIS];      //LSPB trajectory blend time
    double a_const_[NUM_AXIS]; //LSPB constant acceleration
    double inv_tf_;            // =1/tf_
    double waiting_time_;
    double end_time_;
    double pm_[NUM_AXIS]; // for bezier trajectory(middle params)
    int state_;           //arrived(0,1,2...) or moving(-1)
    int state_old_;
    double quintic_coefficient_[NUM_AXIS][6];

    //time-varying
    double p_[NUM_AXIS]; // position
    double v_[NUM_AXIS]; // velocity
    double a_[NUM_AXIS]; // acceleration

    //boolean
    bool pause_;
    bool bezier_init_[NUM_AXIS];
    bool has_final_position;
    bool has_final_velocity;
    bool has_final_acceleration;
    bool has_bezier_middle_point;
    bool has_duration;
    bool is_computed_once;
    bool is_initialize_point;

public:
    TrajectoryGenerator();
    virtual ~TrajectoryGenerator();
    void SetTimeInterval(double dt) { dt_ = dt; }
    void SetTrajectoryType(int type) { trajectory_type_ = type; }
    void SetInitialPosition(double initial_position)
    {
        pi_[SINGLE_AXIS] = initial_position;
        is_initialize_point = true;
    }
    void SetInitialPosition(double initial_x, double initial_y, double initial_z)
    {
        pi_[X] = initial_x;
        pi_[Y] = initial_y;
        pi_[Z] = initial_z;
        trajectory_mode_ = POINT_XYZ;
        is_initialize_point = true;
    }

    void SetBasicParameters(double final_position, double duration_t, double wait_t = 0)
    {
        pf_[SINGLE_AXIS] = final_position;
        tf_ = duration_t;
        waiting_time_ = wait_t;
        has_final_position = true;
        has_duration = true;
        is_computed_once = false;
    }
    void SetBasicParameters(double final_x, double final_y, double final_z, double duration_t, double wait_t = 0)
    {
        pf_[X] = final_x;
        pf_[Y] = final_y;
        pf_[Z] = final_z;
        tf_ = duration_t;
        waiting_time_ = wait_t;
        has_final_position = true;
        has_duration = true;
        is_computed_once = false;
        trajectory_mode_ = POINT_XYZ;
    }
    void SetFinalPosition(double final_position)
    {
        pf_[SINGLE_AXIS] = final_position;
        has_final_position = true;
        is_computed_once = false;
    }
    void SetFinalPosition(double final_x, double final_y, double final_z)
    {
        pf_[X] = final_x;
        pf_[Y] = final_y;
        pf_[Z] = final_z;
        has_final_position = true;
        is_computed_once = false;
        trajectory_mode_ = POINT_XYZ;
    }
    void SetFinalVelocity(double final_velocity)
    {
        vf_[SINGLE_AXIS] = final_velocity;
        has_final_velocity = true;
    }
    void SetFinalVelocity(double final_x_dot, double final_y_dot, double final_z_dot)
    {
        vf_[X] = final_x_dot;
        vf_[Y] = final_y_dot;
        vf_[Z] = final_z_dot;
        has_final_velocity = true;
    }
    void SetFinalAcceleration(double final_acceleration)
    {
        af_[SINGLE_AXIS] = final_acceleration;
        has_final_acceleration = true;
    }
    void SetFinalAcceleration(double final_x_ddot, double final_y_ddot, double final_z_ddot)
    {
        af_[X] = final_x_ddot;
        af_[Y] = final_y_ddot;
        af_[Z] = final_z_ddot;
        has_final_acceleration = true;
    }

    void SetConstantVelocity(double const_velocity)
    {
        v_const_[SINGLE_AXIS] = const_velocity;
    }
    void SetConstantVelocity(double const_x_dot, double const_y_dot, double const_z_dot)
    {
        v_const_[X] = const_x_dot;
        v_const_[Y] = const_y_dot;
        v_const_[Z] = const_z_dot;
    }
    void SetDuration(double duration_t, double wait_t = 0)
    {
        tf_ = duration_t;
        waiting_time_ = wait_t;
        has_duration = true;
        is_computed_once = false;
    }
    void SetBezierMiddlePosition(double bezier_middle_position)
    {
        pm_[SINGLE_AXIS] = bezier_middle_position;
        has_bezier_middle_point = true;
    }
    void SetBezierMiddlePosition(double bezier_mid_x, double bezier_mid_y, double bezier_mid_z)
    {
        pm_[X] = bezier_mid_x;
        pm_[Y] = bezier_mid_y;
        pm_[Z] = bezier_mid_z;
        has_bezier_middle_point = true;
    }

    void ComputeTrajectory();

    int GetState() { return state_; }
    double GetPosition(int axis = SINGLE_AXIS) { return p_[axis]; }
    double GetVelocity(int axis = SINGLE_AXIS) { return v_[axis]; }
    double GetAcceleration(int axis = SINGLE_AXIS) { return a_[axis]; }
    double *GetXYZ() { return p_; }
    double *GetXYZDot() { return v_; }
    double *GetXYZDdot() { return a_; }

    void Resume() { pause_ = false; }
    void Pause() { pause_ = true; }
    void Reset() { state_ = 0; }
    void example();
    void BreakingUp();

private:
    void Initialize();
    void IncreaseTimeAndStateUpdate();
    void FinalValueToInitialValue(int axis = SINGLE_AXIS);
    int Sgn(double val) { return (kEpsilon < val) - (val < -kEpsilon); }
    void SetTempQuinticTraj(const double pi, const double pf, const double vi, const double vf, const double ai, const double af, const double tf, size_t axis);
    void ComputeTrajectoryUsingTime(double time, size_t axis);
};
TrajectoryGenerator::TrajectoryGenerator() : kEpsilon(0.0000001), kPi(3.14159265358979323846)
{
    Initialize();
}
TrajectoryGenerator::~TrajectoryGenerator()
{
    // TODO Auto-generated destructor stub
}
void TrajectoryGenerator::Initialize()
{
    // kEpsilon = 0.0000001;
    trajectory_type_ = SINUSOIDAL; // default type
    state_ = 0;
    state_old_ = 0;
    dt_ = 0.001; // default: 1000hz -> 0.001s
    t_ = 0;
    gt_ = 0;
    trajectory_mode_ = POINT_SINGLE;
    pause_ = false;
    has_final_position = false;
    has_final_velocity = false;
    has_final_acceleration = false;
    has_bezier_middle_point = false;
    has_duration = false;
    is_computed_once = false;

    for (size_t i = 0; i < NUM_AXIS; i++)
    {
        pi_[i] = 0;
        pf_[i] = 0;
        vi_[i] = 0;
        vf_[i] = 0;
        ai_[i] = 0;
        af_[i] = 0;
        bezier_init_[i] = false;
    }
}
void TrajectoryGenerator::ComputeTrajectory()
{
    if (is_computed_once == false)
    {
        if (has_final_position == false)
        {
            std::cout << "You must set a final position" << std::endl;
            return;
        }
        if (has_duration == false)
        {
            std::cout << "You must set a duratinon" << std::endl;
            return;
        }
        if (has_final_velocity == false)
        {
            for (size_t axis = 0; axis < trajectory_mode_; axis++)
                vf_[axis] = 0;
            has_final_velocity = true;
        }
        if (has_final_acceleration == false)
        {
            for (size_t axis = 0; axis < trajectory_mode_; axis++)
                af_[axis] = 0;
            has_final_acceleration = true;
        }

        if (!(tf_ < kEpsilon))
            inv_tf_ = 1 / tf_;
        else
            inv_tf_ = 0;
        end_time_ = tf_ + waiting_time_;
        if (state_ == 0)
            t_ = 0;
        else
            t_ = dt_;
        state_old_ = state_;
        state_ = -1;
        for (size_t axis = 0; axis < trajectory_mode_; axis++)
        {
            bezier_init_[axis] = false;
            if (state_old_ != 0 && is_initialize_point == false)
            {
                FinalValueToInitialValue(axis);
            }

            if (trajectory_type_ != QUINTIC && (fabs(vi_[axis]) > kEpsilon || fabs(ai_[axis]) > kEpsilon || fabs(vf_[axis]) > kEpsilon || fabs(af_[axis]) > kEpsilon))
            {
                trajectory_type_ = QUINTIC;
                quintic_coefficient_[axis][0] = pi_[axis];
                quintic_coefficient_[axis][1] = vi_[axis];
                quintic_coefficient_[axis][2] = ai_[axis] / 2.;
                quintic_coefficient_[axis][3] = (20 * pf_[axis] - 20 * pi_[axis] + tf_ * (af_[axis] * tf_ - 3 * ai_[axis] * tf_ - 8 * vf_[axis] - 12 * vi_[axis])) / (2. * pow(tf_, 3));
                quintic_coefficient_[axis][4] = (-30 * pf_[axis] + 30 * pi_[axis] + tf_ * (-2 * af_[axis] * tf_ + 3 * ai_[axis] * tf_ + 14 * vf_[axis] + 16 * vi_[axis])) / (2. * pow(tf_, 4));
                quintic_coefficient_[axis][5] = (12 * pf_[axis] - 12 * pi_[axis] + tf_ * ((af_[axis] - ai_[axis]) * tf_ - 6 * (vf_[axis] + vi_[axis]))) / (2. * pow(tf_, 5));

                std::cout << "Trajectory type is changed to Quintic" << std::endl;
            }
            else if (trajectory_type_ == QUINTIC)
            {
                quintic_coefficient_[axis][0] = pi_[axis];
                quintic_coefficient_[axis][1] = vi_[axis];
                quintic_coefficient_[axis][2] = ai_[axis] / 2.;
                quintic_coefficient_[axis][3] = (20 * pf_[axis] - 20 * pi_[axis] + tf_ * (af_[axis] * tf_ - 3 * ai_[axis] * tf_ - 8 * vf_[axis] - 12 * vi_[axis])) / (2. * pow(tf_, 3));
                quintic_coefficient_[axis][4] = (-30 * pf_[axis] + 30 * pi_[axis] + tf_ * (-2 * af_[axis] * tf_ + 3 * ai_[axis] * tf_ + 14 * vf_[axis] + 16 * vi_[axis])) / (2. * pow(tf_, 4));
                quintic_coefficient_[axis][5] = (12 * pf_[axis] - 12 * pi_[axis] + tf_ * ((af_[axis] - ai_[axis]) * tf_ - 6 * (vf_[axis] + vi_[axis]))) / (2. * pow(tf_, 5));
            }
            else if (trajectory_type_ == LSPB)
            {
                v_const_[axis] = fabs(v_const_[axis]) * Sgn(pf_[axis] - pi_[axis]);
                //Too small or Too big velocity
                if (fabs(v_const_[axis]) < (fabs(pf_[axis] - pi_[axis]) * inv_tf_) || fabs(v_const_[axis]) > (2 * fabs(pf_[axis] - pi_[axis]) * inv_tf_))
                {
                    v_const_[axis] = 1.5 * (pf_[axis] - pi_[axis]) * inv_tf_;
                    std::cout << "LSPB constant Velocity is changed" << std::endl;
                }
                if (fabs(v_const_[axis]) > kEpsilon)
                    tb_[axis] = (pi_[axis] - pf_[axis] + v_const_[axis] * tf_) / v_const_[axis];
                else
                    tb_[axis] = 0.;
                if (fabs(tb_[axis]) > kEpsilon)
                    a_const_[axis] = v_const_[axis] / tb_[axis];
                else
                    a_const_[axis] = 0.;
            }
        }
        is_computed_once = true;
    }

    for (size_t axis = 0; axis < trajectory_mode_; axis++)
    {
        switch (trajectory_type_)
        {
        case SINUSOIDAL:
            if (t_ < tf_ - kEpsilon) // t_<=tf_
            {
                p_[axis] = (pf_[axis] - pi_[axis]) * 0.5 * (1 - std::cos(kPi * t_ * inv_tf_)) + pi_[axis];
                v_[axis] = (pf_[axis] - pi_[axis]) * 0.5 * std::sin(kPi * t_ * inv_tf_) * kPi * inv_tf_;
                a_[axis] = (pf_[axis] - pi_[axis]) * 0.5 * std::cos(kPi * t_ * inv_tf_) * kPi * kPi * inv_tf_ * inv_tf_;
            }
            else if (t_ < end_time_ + kEpsilon) //duration < t < end_time
            {
                p_[axis] = pf_[axis];
                v_[axis] = vf_[axis];
                a_[axis] = af_[axis];
            }
            break;

        case LSPB:
            if (t_ < (tb_[axis] - kEpsilon))
            {
                p_[axis] = pi_[axis] + a_const_[axis] * 0.5 * t_ * t_;
                v_[axis] = a_const_[axis] * t_;
                a_[axis] = a_const_[axis];
            }
            else if (t_ < (tf_ - tb_[axis] - kEpsilon))
            {
                p_[axis] = (pf_[axis] + pi_[axis] - v_const_[axis] * tf_) * 0.5 + v_const_[axis] * t_;
                v_[axis] = v_const_[axis];
                a_[axis] = 0.;
            }
            else if (t_ < (tf_ - kEpsilon))
            {
                p_[axis] = pf_[axis] - a_const_[axis] * 0.5 * tf_ * tf_ + a_const_[axis] * tf_ * t_ - a_const_[axis] * 0.5 * t_ * t_;
                v_[axis] = a_const_[axis] * tf_ - a_const_[axis] * t_;
                a_[axis] = -a_const_[axis];
            }

            else if (t_ < end_time_ + kEpsilon)
            {
                p_[axis] = pf_[axis];
                v_[axis] = 0.;
                a_[axis] = 0.;
            }
            break;

        case QUINTIC:
            if (t_ < tf_ - kEpsilon) //same as t_<=tf_
            {
                p_[axis] = quintic_coefficient_[axis][0] + quintic_coefficient_[axis][1] * t_ + quintic_coefficient_[axis][2] * t_ * t_ + quintic_coefficient_[axis][3] * t_ * t_ * t_ + quintic_coefficient_[axis][4] * t_ * t_ * t_ * t_ + quintic_coefficient_[axis][5] * t_ * t_ * t_ * t_ * t_;
                v_[axis] = quintic_coefficient_[axis][1] + 2 * quintic_coefficient_[axis][2] * t_ + 3 * quintic_coefficient_[axis][3] * t_ * t_ + 4 * quintic_coefficient_[axis][4] * t_ * t_ * t_ + 5 * quintic_coefficient_[axis][5] * t_ * t_ * t_ * t_;
                a_[axis] = 2 * quintic_coefficient_[axis][2] + 6 * quintic_coefficient_[axis][3] * t_ + 12 * quintic_coefficient_[axis][4] * t_ * t_ + 20 * quintic_coefficient_[axis][5] * t_ * t_ * t_;
            }
            else if (t_ < end_time_ + kEpsilon) //duration < t < end_time
            {
                p_[axis] = pf_[axis];
                v_[axis] = vf_[axis];
                a_[axis] = af_[axis];
            }
            break;

        case BEZIER:
        {
            if (trajectory_mode_ == POINT_XYZ)
            {
                double t_temp = tf_ * 0.1;
                if (t_ < t_temp - kEpsilon)
                {
                    if (bezier_init_[axis] == false)
                    {
                        ComputeTrajectoryUsingTime(t_temp, axis); //Compute p_,v_,a_ at t=t_temp
                        SetTempQuinticTraj(pi_[axis], p_[axis], vi_[axis], v_[axis], ai_[axis], a_[axis], t_temp, axis);
                        bezier_init_[axis] = true;
                    }
                    //Quintic trajectory

                    p_[axis] = quintic_coefficient_[axis][0] + quintic_coefficient_[axis][1] * t_ + quintic_coefficient_[axis][2] * t_ * t_ + quintic_coefficient_[axis][3] * t_ * t_ * t_ + quintic_coefficient_[axis][4] * t_ * t_ * t_ * t_ + quintic_coefficient_[axis][5] * t_ * t_ * t_ * t_ * t_;
                    v_[axis] = quintic_coefficient_[axis][1] + 2 * quintic_coefficient_[axis][2] * t_ + 3 * quintic_coefficient_[axis][3] * t_ * t_ + 4 * quintic_coefficient_[axis][4] * t_ * t_ * t_ + 5 * quintic_coefficient_[axis][5] * t_ * t_ * t_ * t_;
                    a_[axis] = 2 * quintic_coefficient_[axis][2] + 6 * quintic_coefficient_[axis][3] * t_ + 12 * quintic_coefficient_[axis][4] * t_ * t_ + 20 * quintic_coefficient_[axis][5] * t_ * t_ * t_;
                }
                else if (t_ < tf_ - t_temp - kEpsilon) //same as t_<=tf_
                {
                    p_[axis] = pm_[axis] + (1 - t_ * inv_tf_) * (1 - t_ * inv_tf_) * (pi_[axis] - pm_[axis]) + t_ * inv_tf_ * t_ * inv_tf_ * (pf_[axis] - pm_[axis]);
                    v_[axis] = 2 * (1 - t_ * inv_tf_) * (pm_[axis] - pi_[axis]) + 2 * t_ * inv_tf_ * (pf_[axis] - pm_[axis]);
                    a_[axis] = 2 * (pf_[axis] - 2 * pm_[axis] + pi_[axis]);
                }
                else if (t_ < tf_ - kEpsilon)
                {
                    if (bezier_init_[axis] == true)
                    {
                        ComputeTrajectoryUsingTime(tf_ - t_temp, axis); //calculate pos,vel,acc at t=t_temp
                        SetTempQuinticTraj(p_[axis], pf_[axis], v_[axis], vf_[axis], a_[axis], af_[axis], t_temp, axis);
                        bezier_init_[axis] = false;
                    }
                    //Quintic trajectory
                    p_[axis] = quintic_coefficient_[axis][0] + quintic_coefficient_[axis][1] * (t_ - (tf_ - t_temp)) + quintic_coefficient_[axis][2] * (t_ - (tf_ - t_temp)) * (t_ - (tf_ - t_temp)) + quintic_coefficient_[axis][3] * (t_ - (tf_ - t_temp)) * (t_ - (tf_ - t_temp)) * (t_ - (tf_ - t_temp)) + quintic_coefficient_[axis][4] * (t_ - (tf_ - t_temp)) * (t_ - (tf_ - t_temp)) * (t_ - (tf_ - t_temp)) * (t_ - (tf_ - t_temp)) + quintic_coefficient_[axis][5] * (t_ - (tf_ - t_temp)) * (t_ - (tf_ - t_temp)) * (t_ - (tf_ - t_temp)) * (t_ - (tf_ - t_temp)) * (t_ - (tf_ - t_temp));
                    v_[axis] = quintic_coefficient_[axis][1] + 2 * quintic_coefficient_[axis][2] * (t_ - (tf_ - t_temp)) + 3 * quintic_coefficient_[axis][3] * (t_ - (tf_ - t_temp)) * (t_ - (tf_ - t_temp)) + 4 * quintic_coefficient_[axis][4] * (t_ - (tf_ - t_temp)) * (t_ - (tf_ - t_temp)) * (t_ - (tf_ - t_temp)) + 5 * quintic_coefficient_[axis][5] * (t_ - (tf_ - t_temp)) * (t_ - (tf_ - t_temp)) * (t_ - (tf_ - t_temp)) * (t_ - (tf_ - t_temp));
                    a_[axis] = 2 * quintic_coefficient_[axis][2] + 6 * quintic_coefficient_[axis][3] * (t_ - (tf_ - t_temp)) + 12 * quintic_coefficient_[axis][4] * (t_ - (tf_ - t_temp)) * (t_ - (tf_ - t_temp)) + 20 * quintic_coefficient_[axis][5] * (t_ - (tf_ - t_temp)) * (t_ - (tf_ - t_temp)) * (t_ - (tf_ - t_temp));
                }
                else if (t_ < end_time_ + kEpsilon) //duration < t < end_time
                {
                    p_[axis] = pf_[axis];
                    v_[axis] = 0.;
                    a_[axis] = 0.;
                }
            }
            break;
        }
        default:
            std::cout << axis << " Trajectory Unknown\n";
            break;
        }
    }
    IncreaseTimeAndStateUpdate();
}
void TrajectoryGenerator::FinalValueToInitialValue(int axis)
{
    pi_[axis] = p_[axis];
    vi_[axis] = v_[axis];
    ai_[axis] = a_[axis];
}
void TrajectoryGenerator::SetTempQuinticTraj(const double pi, const double pf, const double vi, const double vf, const double ai, const double af, const double tf, size_t axis)
{
    quintic_coefficient_[axis][0] = pi;
    quintic_coefficient_[axis][1] = vi;
    quintic_coefficient_[axis][2] = ai / 2.;
    quintic_coefficient_[axis][3] = (20 * pf - 20 * pi + tf * (af * tf - 3 * ai * tf - 8 * vf - 12 * vi)) / (2. * pow(tf, 3));
    quintic_coefficient_[axis][4] = (-30 * pf + 30 * pi + tf * (-2 * af * tf + 3 * ai * tf + 14 * vf + 16 * vi)) / (2. * pow(tf, 4));
    quintic_coefficient_[axis][5] = (12 * pf - 12 * pi + tf * ((af - ai) * tf - 6 * (vf + vi))) / (2. * pow(tf, 5));
}
void TrajectoryGenerator::ComputeTrajectoryUsingTime(double time, size_t axis)
{
    switch (trajectory_type_)
    {
    case BEZIER:
    {
        if (time < tf_ - kEpsilon) //same as time<=tf_
        {
            p_[axis] = pm_[axis] + (1 - time * inv_tf_) * (1 - time * inv_tf_) * (pi_[axis] - pm_[axis]) + time * inv_tf_ * time * inv_tf_ * (pf_[axis] - pm_[axis]);
            v_[axis] = 2 * (1 - time * inv_tf_) * (pm_[axis] - pi_[axis]) + 2 * time * inv_tf_ * (pf_[axis] - pm_[axis]);
            a_[axis] = 2 * (pf_[axis] - 2 * pm_[axis] + pi_[axis]);
        }

        else if (time < end_time_ + kEpsilon) //duration < time < end_time
        {
            p_[axis] = pf_[axis];
            v_[axis] = 0.;
            a_[axis] = 0.;
        }

        break;
    }
    default:
        std::cout << "Trajectory Unknown\n";
        break;
    }
}
void TrajectoryGenerator::IncreaseTimeAndStateUpdate()
{
    if (t_ < end_time_ - dt_ + kEpsilon && pause_ == false)
    {
        t_ += dt_;
    }
    else if (state_ == -1)
    {
        state_ = state_old_;
        state_++;
        state_old_ = state_;
        has_final_position = false;
        has_final_velocity = false;
        has_final_acceleration = false;
        has_bezier_middle_point = false;
        has_duration = false;
        is_initialize_point = false;
    }
}
void TrajectoryGenerator::BreakingUp()
{
    t_ = end_time_;
    IncreaseTimeAndStateUpdate();
}
#endif /* TRAJECTORYGENERATOR_H_ */
