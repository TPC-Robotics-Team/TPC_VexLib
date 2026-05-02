#pragma once

struct ProfileConstraints
{
    double maxVelocity;
    double maxAcceleration;
};

struct ProfileState
{
    double position;
    double velocity;
};

class TrapezoidalProfile
{
  private:
    ProfileConstraints m_constraints;
    ProfileState m_initial{};
    ProfileState m_goal{};

    double m_direction = 1.0;
    double m_t_accel = 0.0;
    double m_t_cruise = 0.0;
    double m_t_decel = 0.0;
    double m_totalTime = 0.0;
    bool m_generated = false;

  public:
    TrapezoidalProfile(ProfileConstraints constraints);

    void generate(ProfileState initial, ProfileState goal);
    ProfileState sample(double t);
    bool isFinished(double t);
    double totalTime();
};