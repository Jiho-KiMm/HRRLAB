#include "TrajectoryGenerator.h"

void TrajectoryGenerator::init(const double freq)
{
    pi_ = M_PI;
    t_ = 0.0;
    dt_ = 1/freq;

    period_ = 0.0;
    legtype_ = 0;

    qf_.setZero(DoF);
    qi_.setZero(DoF);
    ref_var_.setZero(DoF);
    ref_vardot_.setZero(DoF);
    ref_varddot_.setZero(DoF);
    pre_ref_var_.setZero(DoF);
    pre_ref_vardot_.setZero(DoF);
    pre_ref_varddot_.setZero(DoF);

    walking_flag = 0;

    isEnd = false;
}

void TrajectoryGenerator::SetSinusoidalTrajectory(Eigen::VectorXd qf, Eigen::VectorXd qi, double period)
{
    qf_ = qf;
    qi_ = qi;
    period_ = period;
    t_ += dt_;

    if (t_ >= period_)
    {
        t_ = period_;
        isEnd = true;
    }

    for (size_t i = 0; i < DoF; i++)
    {
        ref_var_(i) = (qf_(i) - qi_(i)) * 0.5 * (1 - cos(1.0 * pi_ * t_ / period_)) + qi_(i);

        ref_vardot_(i) = (qf_(i) - qi_(i)) * 0.5 * (pi_ / period_) * (sin(1.0 * pi_ * t_ / period_));

        ref_varddot_(i) = (qf_(i) - qi_(i)) * 0.5 * pow((pi_ / period_), 2) * (cos(1.0 * pi_ * t_ / period_));
    }
}


void TrajectoryGenerator::SetSinTrajectory(Eigen::VectorXd qf, Eigen::VectorXd qi, double period)
{
    qf_ = qf;
    qi_ = qi;
    period_ = period;
    t_ += dt_;

    if (t_ >= period_)
    {
        t_ = period_;
        isEnd = true;
    }

    for (size_t i = 0; i < DoF; i++)
    {
        ref_var_(i) = (qf_(i) - qi_(i)) * sin(1.0 * pi_ / period_ * t_) + qi_(i);

        ref_vardot_(i) = (qf_(i) - qi_(i)) * (pi_ / period_) * (cos(1.0 * pi_ * t_ / period_));

        ref_varddot_(i) = - (qf_(i) - qi_(i)) * pow((pi_ / period_), 2) * (sin(1.0 * pi_ * t_ / period_));
    }
}

void TrajectoryGenerator::SetWalkingSinusoidalTrajectory(Eigen::VectorXd qf, Eigen::VectorXd qi, double period, int legtype)
{
    qf_ = qf;
    qi_ = qi;
    period_ = period;
    t_ += dt_;

    if (legtype % 2 == 0)
    {
        if (walking_flag % 2 == 0)
        {
            // std::cout << "L1 L3 R2 swing" << legtype << std::endl;
            for (size_t i = 0; i < DoF; i++)
            {
                pre_ref_var_(i) = ref_var_(i);
                pre_ref_vardot_(i) = ref_vardot_(i);
                pre_ref_varddot_(i) = ref_varddot_(i);

                ref_var_(i) = (qf_(i) - qi_(i)) * 0.5 * (1 - cos(1.0 * pi_ * t_ / period_)) + qi_(i);

                ref_vardot_(i) = (qf_(i) - qi_(i)) * 0.5 * (1 * pi_ / period_) * (sin(1.0 * pi_ * t_ / period_));

                ref_varddot_(i) = (qf_(i) - qi_(i)) * 0.5 * pow((1 * pi_ / period_), 2) * (cos(1.0 * pi_ * t_ / period_));

                if (i == 2)
                {
                    ref_var_(i) = (qf_(i) - qi_(i)) * 0.5 * (1 - cos(2.0 * pi_ * t_ / period_)) + qi_(i);

                    ref_vardot_(i) = (qf_(i) - qi_(i)) * 0.5 * (2 * pi_ / period_) * (sin(2.0 * pi_ * t_ / period_));

                    ref_varddot_(i) = (qf_(i) - qi_(i)) * 0.5 * pow((2 * pi_ / period_), 2) * (cos(2.0 * pi_ * t_ / period_));
                }
            }
        }
        else
        {
            // std::cout << "L1 L3 R2 swing" << legtype << std::endl;
            for (size_t i = 0; i < DoF; i++)
            {
                ref_var_(i) = (qf_(i) - qi_(i)) * 0.5 * (1 - cos(1.0 * pi_ * t_ / period_)) + qi_(i);

                ref_vardot_(i) = (qf_(i) - qi_(i)) * 0.5 * (1 * pi_ / period_) * (sin(1.0 * pi_ * t_ / period_));

                ref_varddot_(i) = (qf_(i) - qi_(i)) * 0.5 * pow((1 * pi_ / period_), 2) * (cos(1.0 * pi_ * t_ / period_));

                if (i == 2)
                {
                    ref_var_(i) = qi_(i);

                    ref_vardot_(i) = 0;

                    ref_varddot_(i) = 0;
                }
            }
        }
    }
    else
    {
        if (walking_flag % 2 == 0)
        {
            // std::cout << "L1 L3 R2 swing" << legtype << std::endl;
            for (size_t i = 0; i < DoF; i++)
            {
                if (walking_flag < 2)
                {
                    ref_var_(i) = -(qf_(i) - qi_(i)) * 0.5 * (1 - cos(1.0 * pi_ * t_ / period_)) + qi_(i);

                    ref_vardot_(i) = -(qf_(i) - qi_(i)) * 0.5 * (1 * pi_ / period_) * (sin(1.0 * pi_ * t_ / period_));

                    ref_varddot_(i) = -(qf_(i) - qi_(i)) * 0.5 * pow((1 * pi_ / period_), 2) * (cos(1.0 * pi_ * t_ / period_));
                }
                else
                {
                    ref_var_(i) = -2 * (qf_(i) - qi_(i)) * 0.5 * (1 - cos(1.0 * pi_ * t_ / period_)) + qf_(i);

                    ref_vardot_(i) = -2*(qf_(i) - qi_(i)) * 0.5 * (1 * pi_ / period_) * (sin(1.0 * pi_ * t_ / period_));

                    ref_varddot_(i) = -2*(qf_(i) - qi_(i)) * 0.5 * pow((1 * pi_ / period_), 2) * (cos(1.0 * pi_ * t_ / period_));
                }


                if (i == 2)
                {
                    ref_var_(i) = qi_(i);

                    ref_vardot_(i) = 0;

                    ref_varddot_(i) = 0;
                }
            }
        }
        else
        {
            // std::cout << "L1 L3 R2 swing" << legtype << std::endl;
            for (size_t i = 0; i < DoF; i++)
            {
                ref_var_(i) = -2 * (qf_(i) - qi_(i)) * 0.5 * (1 - cos(1.0 * pi_ * t_ / period_)) + qf_(i);

                ref_vardot_(i) = -2*(qf_(i) - qi_(i)) * 0.5 * (1 * pi_ / period_) * (sin(1.0 * pi_ * t_ / period_));

                ref_varddot_(i) = -2*(qf_(i) - qi_(i)) * 0.5 * pow((1 * pi_ / period_), 2) * (cos(1.0 * pi_ * t_ / period_));

                if (i == 2)
                {
                    ref_var_(i) = (qf_(i) - qi_(i)) * 0.5 * (1 - cos(2.0 * pi_ * t_ / period_)) + qi_(i);

                    ref_vardot_(i) = (qf_(i) - qi_(i)) * 0.5 * (2 * pi_ / period_) * (sin(2.0 * pi_ * t_ / period_));

                    ref_varddot_(i) = (qf_(i) - qi_(i)) * 0.5 * pow((2 * pi_ / period_), 2) * (cos(2.0 * pi_ * t_ / period_));
                }
            }
        }
    }
    if (fmod(t_, period_) < 0.005)
    {
        walking_flag++;
    }
}

void TrajectoryGenerator::ResetPeriod()
{
    t_ = 0;
}