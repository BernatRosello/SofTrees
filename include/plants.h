#pragma once

#include <box2d/id.h>
#include <raylib.h>
#include <vector>

enum ParticleType {
    NON_GROWING_TIP,
    GROWING_TIP,
    BRANCH,
    FLOWER,
    FRUIT,
};

struct Particle {
    b2BodyId bodyId = b2_nullBodyId; // Box2D body
    float length;
    float thickness;
    Color color;
    int parentIndex;
    ParticleType type;
    int level;
};

enum PlantStage {
    SEEDLING = 30,
    JUVENILE = 70,
    MATURE = 110,
    FLOWERING = 170,
    FRUITING = 210,
    DECAY
};

struct Plant {
    std::vector<Particle> particles;
    Color primaryColor;
    Color secondaryColor;
    int maxSize;
    int age = 0;
    
    // Growth parameters
    float tipAngleVariance = 0.15f; // in radians
    float branchingAngle = 0.6f; // in radians
    float branchAngleVariance = 0.05f; // in radians
    float branchProbability = 0.33f;
    float secondaryGrowthRate = 0.1f; // in units per tick

    // Secondary Growth Malleability
    float deformationThreshold = 4 * DEG2RAD;
    float resistanceThreshold = 6 * DEG2RAD;
    float yieldThreshold = 20 * DEG2RAD;
};

static inline PlantStage GetPlantStage(Plant& plant) {
    PlantStage res = PlantStage::DECAY;
    int age = plant.age;
    
    if (age<=PlantStage::SEEDLING) {
        res = PlantStage::SEEDLING;
    } else if (age<=PlantStage::JUVENILE) {
        res = PlantStage::JUVENILE;
    } else if (age<=PlantStage::MATURE) {
        res = PlantStage::MATURE;
    } else if (age<=PlantStage::FLOWERING) {
        res = PlantStage::FLOWERING;
    } else if (age<=PlantStage::FRUITING) {
        res = PlantStage::FRUITING;
    }

    return res;
}


struct GrowthStageModifiers {
    float tipGrowthRateMultiplier;
    float branchProbabilityMultiplier;
    float tipAngleVarianceMultiplier;
    float lengthMultiplier;
    float thicknessMultiplier;
    float secondaryGrowthRateMultiplier;
};

static inline GrowthStageModifiers GetGrowthModifiers(PlantStage stage) {
    switch (stage) {
        case SEEDLING:
            return { 1.5f, 0.5f, 0.7f, 1.1f, 0.9f, 1.0f }; // Taller, fewer branches, straighter
        case JUVENILE:
            return { 1.0f, 1.0f, 1.0f, 1.0f, 1.2f, 1.1f }; // Baseline growth
        case MATURE:
            return { 0.8f, 1.5f, 1.2f, 0.9f, 0.9f, 0.8f }; // Slower tips, more branching
        case FLOWERING:
        case FRUITING:
            return { 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.8f }; // Growth stops, future: flowers/fruits
        default:
            return { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
    }
}

Particle CreateParticle(b2WorldId worldId, Vector2 pos, float angle, float length, float thickness, Color color, int parentIndex, const Particle* parent, ParticleType growthType);
float GetTropismAngle(Plant& plant, float referenceAngle, Vector2 worldPos, b2WorldId worldId);
void DrawPlant(const Plant& plant, float lineBoilAmplitude=0.25, float lineBoilSpeed=2, float boilThicknessFactor=0.6, float baseOffset=0.05, float phaseScaling=0.667);
void GrowPlant(Plant &plant, b2WorldId worldId);
float GetDeformationReactionFactor(float deformationtAngle, Plant& p);
void DeletePlant(Plant& plant);


Plant* GetPlantAtPoint(const std::vector<Plant>& plants, b2WorldId worldId, Vector2 worldPos);