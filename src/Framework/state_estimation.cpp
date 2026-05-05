#include "custom/state_estimation.hpp"
#include "custom/utils.hpp"

// Constructor
EncoderOdometry::EncoderOdometry(
    pros::MotorGroup& leftMotors, pros::MotorGroup& rightMotors, double wheelDiam, double trackWidth, double gearRatio)
    : m_leftMotorGroup(leftMotors), m_rightMotorGroup(rightMotors), m_wheelDiam(wheelDiam), m_trackWidth(trackWidth),
      m_gearRatio(gearRatio)
{
    m_lastLeft = averageVector(m_leftMotorGroup.get_position_all());
    m_lastRight = averageVector(m_rightMotorGroup.get_position_all());
    m_lastTime = pros::millis();
}

void EncoderOdometry::update()
{
    std::vector<double> leftPositions = m_leftMotorGroup.get_position_all();
    std::vector<double> rightPositions = m_rightMotorGroup.get_position_all();
    double now = pros::millis();

    double avgLeft = averageVector(leftPositions);
    double avgRight = averageVector(rightPositions);

    double dLeft = avgLeft - m_lastLeft;
    double dRight = avgRight - m_lastRight;
    double dt = (now - m_lastTime) / 1000; // Converts to seconds

    double dL = degreesToCm(dLeft, m_wheelDiam) / m_gearRatio;
    double dR = degreesToCm(dRight, m_wheelDiam) / m_gearRatio;

    double dC = (dL + dR) / 2;
    double dHeading = (dR - dL) / m_trackWidth;

    if (fabs(dHeading) > 1e-6)
    {
        double r = dC / dHeading; // Radius of curvature
        m_pose.x += r * (sin(m_pose.heading + dHeading) - sin(m_pose.heading));
        m_pose.y += r * (-cos(m_pose.heading + dHeading) + cos(m_pose.heading));
    }
    else
    {
        m_pose.x += dC * cos(m_pose.heading + dHeading / 2.0);
        m_pose.y += dC * sin(m_pose.heading + dHeading / 2.0);
    }
    m_pose.heading += dHeading;

    m_velocity = (dt > 0) ? (dC / dt) : 0.0;

    m_lastLeft = avgLeft;
    m_lastRight = avgRight;
    m_lastTime = now;
}

void EncoderOdometry::reset()
{
    m_pose = {0,0,0};

    m_lastLeft = averageVector(m_leftMotorGroup.get_position_all());
    m_lastRight = averageVector(m_rightMotorGroup.get_position_all());
    m_lastTime = pros::millis();
}

void EncoderOdometry::setPose(Pose pose)
{
    m_pose = pose;
    m_lastLeft = averageVector(m_leftMotorGroup.get_position_all());
    m_lastRight = averageVector(m_rightMotorGroup.get_position_all());
    m_lastTime = pros::millis();
}

Pose EncoderOdometry::getPose() const
{
    return m_pose;
}

double EncoderOdometry::getVelocity()
{
    return m_velocity;
}
