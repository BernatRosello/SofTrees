#pragma once

static inline float HashFloat(int seed) {
    // Simple hash to float in [0, 1]
    seed = (seed << 13) ^ seed;
    return (1.0f - ((seed * (seed * seed * 15731 + 789221) + 1376312589) & 0x7fffffff) / 1073741824.0f);
}