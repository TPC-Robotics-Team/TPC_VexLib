#include "utils.hpp"
#include "controller.hpp"
#include "pros/imu.hpp"
#include <algorithm>
#include <cmath>


// Slew Delimitation Class
SlewLimiter::SlewLimiter(double accel, double decel){
    this->accel = accel;
    this->decel = decel;
    prev = 0;
}

double SlewLimiter::update(double input) {
    double delta = input - prev;

    if ((input * prev >= 0) && std::abs(input) > std::abs(prev)){
        if (delta > accel) delta = accel;
        else if (delta < -accel) delta = -accel;
    }
    else {
        if (delta > decel) delta = decel;
        else if (delta < -decel) delta = -decel;
    }
    
    prev += delta;

    return prev;
}

// Update the SlewLimiter
void SlewLimiter::reset(double value){
    prev = value;
}


// PID
PID::PID(double p, double i, double d, double iZone, double iMax, double outputLimit)
: kP(p), kI(i), kD(d), integral(0),
  integralZone(iZone), integralLimit(iMax),
  prevMeasurement(0), outputLimit(outputLimit)  {}

double PID::update(double target, double current){
    double error = target - current;

    if (std::abs(error) < integralZone || integralZone == 0){
        integral += error;
    }
    else integral = 0;

    integral = std::clamp(integral, -integralLimit, integralLimit);

    double derivative = current - prevMeasurement;

    prevMeasurement = current;

    double output = kP * error + kI * integral - kD * derivative;

    return std::clamp(output, -outputLimit, outputLimit);
}

void PID::reset(){
    integral = 0;
    prevMeasurement = 0;
}



// Heading Hold
HeadingHold::HeadingHold(pros::Imu &imu, PID pid)
: imu(imu), controller(pid), targetHeading(0), enabled(false) {}


double HeadingHold::update(double throttleInput, double steerInput){

    const double deadband = 10;

    double heading = imu.get_rotation();

    if ((std::abs(steerInput) > deadband)){
        controller.reset();
        return steerInput;
    }

    if (!enabled){
        targetHeading = heading;
        controller.reset();
        enabled = true;
    }

    double error = wrapAngle(targetHeading - heading);

    double correction = controller.update(0, -error);

    return correction;
}

void HeadingHold::setTarget(double target){
    targetHeading = target;
}

void HeadingHold::reset(){
    targetHeading = imu.get_rotation();
    controller.reset();
    enabled = false;
}



// Feedforward
FeedForward::FeedForward(double kS, double kV, double kA){
    this->kS = kS;
    this->kV = kV;
    this->kA = kA;
}


double FeedForward::sign(double velocity){
    if (velocity > 0.0) return 1.0;
    else if (velocity < 0.0) return -1.0;
    return 0.0;
}

double FeedForward::calculate(double velocity, double accel){
    if (velocity == 0.0 and accel == 0.0) return 0.0;

    double output = 0.0;
    output += kS * sign(velocity) + kV * velocity + kA * accel;

    return output;
}



// Encoders
Encoder::Encoder(pros::MotorGroup& motors, double wheelDiam, double gearRatio)
    : m_motorGroup(motors),
      m_wheelCirc(wheelDiam * M_PI),
      m_gearRatio(gearRatio) {}

double Encoder::degreesToCm(double degrees){
    return (degrees / 360) * m_gearRatio * m_wheelCirc;
}

double Encoder::getPosition(){
    std::vector<double> positions = m_motorGroup.get_position_all();
    double sum = 0.0;
    for (double p : positions) sum += p;
    return degreesToCm(sum / positions.size());
}

double Encoder::getDelta(){
    double current = getPosition();
    double delta = current - m_lastPosition;
    m_lastPosition = current;
    return delta;
}

void Encoder::reset(){
    m_motorGroup.tare_position_all();
    m_lastPosition = 0.0;
}



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