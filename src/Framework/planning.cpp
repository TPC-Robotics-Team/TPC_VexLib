#include "custom/planning.hpp"
#include <cmath>
#include <algorithm>


TrapezoidalProfile::TrapezoidalProfile(ProfileConstraints constraints)
    : m_constraints(constraints) {}


void TrapezoidalProfile::generate(ProfileState initial, ProfileState goal){
    m_initial = initial;
    m_goal = goal;

    double delta_pos = goal.position - initial.position;
    m_direction = (delta_pos >= 0) ? 1.0 : -1.0;
    double dist = std::abs(delta_pos);

    double vMax = m_constraints.maxVelocity;
    double a = m_constraints.maxAcceleration;

    double v0 = initial.velocity * m_direction;
    double vf = goal.velocity * m_direction;

    m_t_accel = (vMax - v0) / a;
    m_t_decel = (vMax - vf) / a;

    double d_accel = (v0 + vMax) / 2.0 * m_t_accel;
    double d_decel = (vf + vMax) / 2.0 * m_t_decel;

    double d_cruise = dist - d_accel - d_decel;

    if (d_cruise < 0){
        double v_peak = std::sqrt((2.0 * a * dist + v0 * v0 + vf * vf) / 2.0);
        v_peak = std::min(v_peak, vMax);

        m_t_accel = (v_peak - v0) / a;
        m_t_decel = (v_peak - vf) / a;
        m_t_cruise = 0.0;
    }
    else{
        m_t_cruise = d_cruise / vMax;
    }

    m_totalTime = m_t_accel + m_t_cruise + m_t_decel;
    m_generated = true;
}


ProfileState TrapezoidalProfile::sample(double t){
    if (!m_generated) return m_initial;

    t = std::clamp(t, 0.0, m_totalTime);

    double a = m_constraints.maxAcceleration;
    double v0 = m_initial.velocity * m_direction;

    double v_peak = v0 + a * m_t_accel;

    double pos, vel;

    if (t <= m_t_accel){
        vel = v0 + a * t;
        pos = v0 * t + 0.5 * a * t * t;
    }
    else if(t <= m_t_accel + m_t_cruise){
        double dt = t - m_t_accel;
        double d_accel = v0 * m_t_accel + 0.5 * m_t_accel * m_t_accel;
        vel = v_peak;
        pos = d_accel + v_peak * dt;
    }
    else{
        double dt = t - m_t_accel - m_t_cruise;
        double d_accel = v0 * m_t_accel + 0.5 * m_t_accel * m_t_accel;
        double d_cruise = v_peak * m_t_cruise;
        vel = v_peak - a * dt;
        pos = d_accel + d_cruise + v_peak * dt - 0.5 * a * dt * dt;
    }
    return {
        m_initial.position + m_direction * pos,
        m_direction * vel
    };
}

bool TrapezoidalProfile::isFinished(double t){
    return m_generated && t >= m_totalTime;
}

double TrapezoidalProfile::totalTime(){
    return m_totalTime;
}