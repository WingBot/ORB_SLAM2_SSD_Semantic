#ifndef DUTILS_RANDOM_H
#define DUTILS_RANDOM_H

#include <cstdlib>
#include <ctime>

namespace DUtils
{
    class Random
    {
    public:
        static void SeedRand(int seed = -1)
        {
            if (seed == -1)
                srand(time(NULL));
            else
                srand(seed);
        }
        
        static int RandomInt(int min, int max)
        {
            return min + rand() % (max - min + 1);
        }
        
        static float RandomFloat(float min, float max)
        {
            return min + (float)rand() / RAND_MAX * (max - min);
        }
    };
}

#endif
