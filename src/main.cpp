#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <chrono>
#include "Body.hpp"

// debug
void set3body(Body* b)
{
    b[0].setInitialConditions(0, 1.0f, -1.0f, 0.0f, 0.0f, 0.0f,  0.3f, 0.0f);
    b[1].setInitialConditions(1, 1.0f,  1.0f, 0.0f, 0.0f, 0.0f, -0.3f, 0.0f);
    b[2].setInitialConditions(2, 1.0f,  0.0f, 0.5f, 0.0f, 0.0f,  0.0f, 0.0f);
}

// Init AoS with random values
void rndInit(Body* b, uint32_t num_bodies)
{
    // Set seed for reproducibility
    std::srand(42);

    // Body 0 is on center with no initial velocity
    b[0].setInitialConditions(0, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    // Generate random bodies around the center
    for (uint32_t i = 1; i < num_bodies; ++i)
    {
        b[i].pid = i;
        b[i].mass = 1.0f;
        b[i].pos[0] = static_cast<float>(std::rand()) / RAND_MAX * 100.0f - 50.0f; // Random x in [-50, 50]
        b[i].pos[1] = static_cast<float>(std::rand()) / RAND_MAX * 100.0f - 50.0f; // Random y in [-50, 50]
        b[i].pos[2] = static_cast<float>(std::rand()) / RAND_MAX * 100.0f - 50.0f; // Random z in [-50, 50]
        // Initial velocity is zero for all bodies
        b[i].vel[0] = 0.0f;
        b[i].vel[1] = 0.0f;
        b[i].vel[2] = 0.0f;
    }
}

void printBodyInfo(const Body& b)
{
    printf("Body %u: Mass=%.2f, Pos=(%.2f, %.2f, %.2f), Vel=(%.2f, %.2f, %.2f)\n",
           b.pid, b.mass, b.pos[0], b.pos[1], b.pos[2], b.vel[0], b.vel[1], b.vel[2]);
}

// Requires arg for number of bodies, time step, and number of steps
int main(int argc, char* argv[])
{
    // Check for correct number of arguments
    if (argc != 4)
    {
        std::cerr << "Usage: " << argv[0] << " <num_bodies> <time_step> <num_steps>" << std::endl;
        return EXIT_FAILURE;
    }

    // Parse arguments
    uint32_t num_bodies = std::atoi(argv[1]);
    uint32_t num_steps = std::atoi(argv[3]);
    float time_step = std::atof(argv[2]);

    // Create an array of empty bodies
    Body* bodies = (Body*)malloc(num_bodies * sizeof(Body));

    // Initialize bodies
    float G = 1.0f;
    if (num_bodies == 3)
    {
        set3body(bodies);
    }
    else
    {
        rndInit(bodies, num_bodies);
    }

    // Simulation loop
    for (uint32_t step = 0; step < num_steps+1; ++step)
    {
        auto start_ts = std::chrono::high_resolution_clock::now();
        printf("Step %u:\n", step);
        // Forces
        auto start_f = std::chrono::high_resolution_clock::now();
        for (uint32_t i = 0; i < num_bodies; ++i)
        {
            // Reset acceleration for body i
            bodies[i].acc[0] = bodies[i].acc[1] = bodies[i].acc[2] = 0.0f;

            // Compute acceleration from all other bodies
            for (uint32_t j = 0; j < num_bodies; ++j)
            {
                if (i != j)
                {
                    bodies[i].computeAccel_IJ(bodies[j], G);
                }
            }
        }
        auto stop_f = std::chrono::high_resolution_clock::now();
        auto duration_f = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_f - start_f);
        std::cout << "Force computation time: " << duration_f.count() << " ns" << std::endl;

        // Updates
        auto start_u = std::chrono::high_resolution_clock::now();
        for (uint32_t i = 0; i < num_bodies; ++i)
        {
            bodies[i].update(time_step);
        }
        auto stop_u = std::chrono::high_resolution_clock::now();
        auto duration_u = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_u - start_u);
        std::cout << "Update time: " << duration_u.count() << " ns" << std::endl;

        auto stop_ts = std::chrono::high_resolution_clock::now();
        auto duration_ts = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_ts - start_ts);
        std::cout << "Total step time: " << duration_ts.count() << " ns" << std::endl;
        printBodyInfo(bodies[0]); // Print info for body 0 at each step
    }

    free(bodies);
    return 0;
}