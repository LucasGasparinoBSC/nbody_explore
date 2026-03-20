#ifndef BODY_HPP
#define BODY_HPP

#include <cstdint>
#include <cmath>
#include <ctime>

class Body
{
    public:
        uint32_t pid;
        float mass;
        float pos[3];
        float vel[3];
        float acc[3];

        // constructors
        Body();
        Body(uint32_t id, float m, float x, float y, float z, float vx, float vy, float vz);
        Body(const Body& other);
        ~Body();

        void setInitialConditions(uint32_t id, float m, float x, float y, float z, float vx, float vy, float vz);
        void computeAccel_IJ(const Body& other, const float G);
        void update(const float dt);
};

#endif