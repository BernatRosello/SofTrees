#include "plants.h"

#include "raylib.h"
#include "raymath.h"
#include "b2_utils.h"
#include "hash.h"
#include "random.h"
#include <iostream>

void DeletePlant(Plant& plant) {
    for (auto& particle : plant.particles) {
        if (b2Body_IsValid(particle.bodyId)) {
            b2DestroyBody(particle.bodyId);
            particle.bodyId = b2_nullBodyId;
        }
    }
    plant.particles.clear();
}

Particle CreateParticle(b2WorldId worldId, Vector2 pos, float angle, float length, float thickness, Color color, int parentIndex, const Particle* parent, ParticleType growthType) {
    b2BodyDef bd = b2DefaultBodyDef();
    b2ShapeDef shapeDef = b2DefaultShapeDef();
    // emualte mass of 3D cylinder
    shapeDef.density = thickness/10;

    bd.type = (parentIndex != -1) ? b2_dynamicBody : b2_staticBody;    // or dynamicBody if you want physics interaction
    bd.position = b2Vec2{pos.x, pos.y};
    bd.rotation = b2MakeRot(-PI/2);

    b2Capsule capsule = {b2Vec2{0, 0}, b2Vec2{length,0}, thickness/2};

    // Create a box fixture representing the anisotropic segment (capsule approx)
    b2BodyId bodyId = b2CreateBody(worldId, &bd);
    b2CreateCapsuleShape(bodyId, &shapeDef, &capsule);

    if (parentIndex != -1)
    {
        b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
        jointDef.bodyIdA = parent->bodyId;
        jointDef.bodyIdB = bodyId;
        jointDef.localAnchorA = {parent->length,0};
        jointDef.localAnchorB = {0,0};
        jointDef.targetAngle = angle;
        jointDef.enableSpring = true;
        jointDef.hertz = length/4 * thickness*1.5f  + 20;
        jointDef.dampingRatio = 0.995f;
        jointDef.collideConnected = false;
        b2CreateRevoluteJoint(worldId, &jointDef);
    }

    int level = (parentIndex != -1) ? parent->level + 1 : 0;

    Particle p = {
        bodyId,
        length,
        thickness,
        color,
        parentIndex,
        growthType,
        level
    };

    return p;
}

float GetTropismAngle(Plant& plant, float referenceAngle, Vector2 worldPos, b2WorldId worldId) {
    // Gravity tropism pulls downward
    float gravityBias = 0.0f; // Straight up
    auto b2WorldPos = b2Vec2{worldPos.x, worldPos.y};

    // Separation tropism: find nearby growing tips
    b2Vec2 repulsion = { 0.0f, 0.0f };
    const float repulsionStrength = 1.0f;

    auto closeby = GetBodiesWithinRadius(worldId, b2WorldPos, 40, 20);
    int neighborCount = closeby.size();
    for (b2BodyId otherId : closeby) {
        auto otherPos = b2Body_GetPosition(otherId);
        repulsion += b2WorldPos-otherPos;
    }
    

    float repulsionAngle = (neighborCount > 0) ? atan2f(repulsion.y, repulsion.x) * repulsionStrength : referenceAngle;

    // Weight factors (tweakable)
    float gravityWeight = 0.0f;
    float separationWeight = 1.0f;

    // Combine influences
    float targetAngle = referenceAngle;

    // Pull toward gravity
    targetAngle = gravityBias;//Lerp(targetAngle, gravityBias, gravityWeight);

    // Push away from neighbors
    if (neighborCount > 0) {
        targetAngle = Lerp(targetAngle, repulsionAngle, separationWeight);
    }
    std::cout << "neighbor count: " << neighborCount << std::endl;

    return targetAngle;
}

void GrowPlant(Plant& plant, b2WorldId worldId) {
    std::vector<Particle> newParticles;

    plant.age++;
    auto stage = GetPlantStage(plant);
    if (stage == PlantStage::DECAY) return;

    auto modifiers = GetGrowthModifiers(stage); // You can adjust this per stage for growth biasing

    for (size_t i = 0; i < plant.particles.size(); i++) {

        Particle& p = plant.particles[i];

        // Common growth parameters
        float lerpStep = 0.5f * 1 / p.thickness;
        Color newColor = ColorLerp(p.color, plant.secondaryColor, lerpStep);

        b2ShapeId shapes[1];
        b2Body_GetShapes(p.bodyId, shapes, 1);
        auto shapeId = shapes[0];
        b2Capsule segmentCapsule = b2Shape_GetCapsule(shapes[0]);

        Vector2 basePos = rayVec2(b2Body_GetWorldPoint(p.bodyId, segmentCapsule.center1));
        Vector2 tipPos = rayVec2(b2Body_GetWorldPoint(p.bodyId, segmentCapsule.center2));

        switch(p.type) {
        case ParticleType::BRANCH:
            {
                // Thicken branches
                p.thickness += plant.secondaryGrowthRate;
                segmentCapsule.radius = p.thickness/2;
                b2Shape_SetCapsule(shapeId, &segmentCapsule);
                b2Body_ApplyMassFromShapes(p.bodyId);

                // Strengthen branch connection and "aneal" connection angle
                b2JointId joints[5]; // 5 is chosen as an ARBITRARY max joint n.
                b2Body_GetJoints(p.bodyId, joints, 5);
                for (auto jointId : joints) {
                    if (b2Joint_IsValid(jointId) && b2Joint_GetType(jointId) == b2JointType::b2_revoluteJoint) {
                        // Deformation anealing
                        float prevTargetAngle = b2RevoluteJoint_GetTargetAngle(jointId);
                        float currentAngle = b2RevoluteJoint_GetAngle(jointId);
                        // Tropism Influence
                        // "minimize" angles that point to the floor
                        //if (abs(currentAngle) > PI/2) currentAngle = Lerp(prevTargetAngle, currentAngle, 0.2f); 
                        //float newTargetAngle = Lerp(prevTargetAngle, currentAngle, abs(currentAngle-prevTargetAngle)>(1-modifiers.secondaryGrowthRateMultiplier) ? 1 : 0 * modifiers.secondaryGrowthRateMultiplier);

                        float newTargetAngle = Lerp(prevTargetAngle, currentAngle, 0.33f * GetDeformationReactionValue(currentAngle, plant) * plant.secondaryGrowthRate);
                        
                        b2RevoluteJoint_SetTargetAngle(jointId, newTargetAngle);

                        float currentStiffness = b2RevoluteJoint_GetSpringHertz(jointId);
                        // Derivative of what would be 'p.thickness^2 / 5'  i think?
                        float newStiffness = std::min(currentStiffness + 2 * plant.secondaryGrowthRate / 20, 20000.0f);
                        b2RevoluteJoint_SetSpringHertz(jointId, newStiffness);
                    }
                }
            }
            break;

        case ParticleType::GROWING_TIP:
        
            // Thicken branches
            p.thickness += plant.secondaryGrowthRate;
            segmentCapsule.radius = p.thickness/2;
            b2Shape_SetCapsule(shapeId, &segmentCapsule);
            b2Body_ApplyMassFromShapes(p.bodyId);

            if (plant.particles.size() + newParticles.size() < plant.maxSize)
            {
                // Propagate and branch
                // Update current tip
                //float baseAngle = b2Rot_GetAngle(b2Body_GetRotation(p.bodyId));
                //float tipAngle = baseAngle-GetTropismAngle(plant, baseAngle, tipPos, worldId);
                float tipAngle = RandomFloatRange(-plant.tipAngleVariance, plant.tipAngleVariance);
                Particle newParticle = CreateParticle(worldId,
                    tipPos,
                    tipAngle,
                    p.length * RandomFloatRange(0.9f, 1.1f), 
                    p.thickness * RandomFloatRange(0.85f,0.99f), 
                    newColor, 
                    (int)i,
                    &p,
                    ParticleType::GROWING_TIP);

                newParticles.push_back(newParticle);
                p.type = ParticleType::BRANCH;

                // Random lateral branch
                if (RandomFloatRange(0.0f, 1.0f) < plant.branchProbability) {
                    float branchAngle = (p.level%2 ? 1 : -1)*plant.branchingAngle + RandomFloatRange(-plant.branchAngleVariance, plant.branchAngleVariance);
                    Particle branch = CreateParticle(worldId,
                        basePos,
                        branchAngle,
                        p.length * RandomFloatRange(0.9f, 1.1f),
                        p.thickness * RandomFloatRange(0.75f,0.8f),
                        newColor,
                        (int)i,
                        &p,
                        ParticleType::GROWING_TIP);
                    newParticles.push_back(branch);
                }
            }
            break;
        default:
            {
                std::cout << "yep, nothing growin" << std::endl;
            }
            break;
        }
    }

    plant.particles.insert(plant.particles.end(), newParticles.begin(), newParticles.end());
}

float GetDeformationReactionValue(float deformationAngle, Plant& p)
{
    float da = abs(deformationAngle);
    const float dir = deformationAngle < 0 ? -1 : 1;
    if (da < p.deformationThreshold)
        return 0;
    else if (da < p.resistanceThreshold)
        return -da/(p.resistanceThreshold-p.deformationThreshold);
    else if (da < p.yieldThreshold)
        //return 0;
        return (da-p.resistanceThreshold)/(p.yieldThreshold-p.resistanceThreshold)*2 - 1 ;
    else
        return 1;
}

void DrawPlant(const Plant& plant, float lineBoilAmplitude, float lineBoilSpeed, float boilThicknessFactor, float baseOffset, float phaseScaling) {
    double time = GetTime();

    for (size_t i = 0; i < plant.particles.size(); i++) {
        const Particle& p = plant.particles[i];
        b2ShapeId shapes[1];
        b2Body_GetShapes(p.bodyId, shapes, 1);
        b2Capsule segmentCapsule = b2Shape_GetCapsule(shapes[0]);
        Vector2 base = rayVec2(b2Body_GetWorldPoint(p.bodyId, segmentCapsule.center1));//-b2Vec2{baseOffset*p.length,0}));
        Vector2 tip = rayVec2(b2Body_GetWorldPoint(p.bodyId, segmentCapsule.center2));

        Vector2 center = Vector2Lerp(base, tip, 0.5f);
        float len = Vector2Length(Vector2Subtract(base, tip));
        Vector2 perp = {(base.y - tip.y) / len, -(base.x - tip.x) / len};

        switch (p.type)
        {
        case ParticleType::FLOWER:
            DrawTriangle(base, center+perp, center-perp, p.color);
            break;

        case ParticleType::FRUIT:
            DrawRectangle(center.x, center.y, len, len, p.color);
            break;

        default:
            // Use hash-based pseudo-random phase from particle index
            double phase = (p.level) * PI/phaseScaling + HashFloat(b2StoreBodyId(plant.particles[0].bodyId))/4;


            //Vector2 startpos = Vector2Lerp(base, tip, sinf(time * 5 + phase + 1) * 0.1f - 0.1f);
            //Vector2 endpos = Vector2Lerp(base, tip, sinf(time * 5 + phase) * 0.1f + 0.9f);

            float thickFactor = Clamp((plant.particles[0].thickness-p.thickness)/(boilThicknessFactor*plant.particles[0].thickness), 0, 1);
            float boilAmplitude = lineBoilAmplitude * len * thickFactor;
            float boilSpeed = lineBoilSpeed * thickFactor;
            float boilOffset = sinf(time * boilSpeed + phase) * boilAmplitude;

            Vector2 cppos = center + perp * boilOffset;

            //DrawSplineSegmentBezierQuadratic(startpos, cppos, endpos, segmentCapsule.radius, p.color);
            DrawSplineSegmentBezierQuadratic(base, cppos, tip, segmentCapsule.radius*2, p.color);
            DrawCircle(base.x, base.y, segmentCapsule.radius, p.color);
            DrawCircle(tip.x, tip.y, segmentCapsule.radius, p.color);
        }
    }
}

Plant* GetPlantAtPoint(const std::vector<Plant>& plants, b2WorldId worldId, Vector2 worldPos) {
    b2BodyId bodyUnderCursor = GetBodyAtPoint(worldId, worldPos);
    if (B2_IS_NULL(bodyUnderCursor)) return nullptr;

    for (const Plant& plant : plants) {
        for (const Particle& p : plant.particles) {
            if (B2_ID_EQUALS(p.bodyId, bodyUnderCursor)) {
                return (Plant*)&plant;
            }
        }
    }
    return nullptr;
}