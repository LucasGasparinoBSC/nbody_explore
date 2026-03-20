#include "Body.hpp"

Body::Body()
{
    pid = 0;
    mass = 0.0f;
    pos[0] = pos[1] = pos[2] = 0.0f;
    vel[0] = vel[1] = vel[2] = 0.0f;
    acc[0] = acc[1] = acc[2] = 0.0f;
}

Body::Body(uint32_t id, float m, float x, float y, float z, float vx, float vy, float vz)
{
    setInitialConditions(id, m, x, y, z, vx, vy, vz);
}

Body::~Body()
{
    // No dynamic memory to clean up
}

void Body::setInitialConditions(uint32_t id, float m, float x, float y, float z, float vx, float vy, float vz)
{
    pid = id;
    mass = m;
    pos[0] = x; pos[1] = y; pos[2] = z;
    vel[0] = vx; vel[1] = vy; vel[2] = vz;
    acc[0] = acc[1] = acc[2] = 0.0f;
}

void Body::computeAccel_IJ(const Body& other, float G)
{
    float rij[3];

    // Compute distance rij = rj - ri
    rij[0] = other.pos[0] - pos[0];
    rij[1] = other.pos[1] - pos[1];
    rij[2] = other.pos[2] - pos[2];

    // Compute distance squared
    float r2 = rij[0] * rij[0] + rij[1] * rij[1] + rij[2] * rij[2];

    // Denom: (r2 + eps^2)^(3/2)
    float eps = 0.2f;
    float denom = r2 + eps*eps;
    denom = 1.0f / sqrtf(denom);
    float denom3 = denom * denom * denom; // (r2 + eps^2)^(-3/2)

    // Compute acceleration contribution
    acc[0] += G * other.mass * rij[0] * denom3;
    acc[1] += G * other.mass * rij[1] * denom3;
    acc[2] += G * other.mass * rij[2] * denom3;
}

void Body::update(float dt)
{
    // Update velocity: v = v + a * dt
    vel[0] += acc[0] * dt;
    vel[1] += acc[1] * dt;
    vel[2] += acc[2] * dt;

    // Update position: r = r + v * dt
    pos[0] += vel[0] * dt;
    pos[1] += vel[1] * dt;
    pos[2] += vel[2] * dt;
}