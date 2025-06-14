#include "raylib.h"
#include "raymath.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#include "box2d/box2d.h"

#include <vector>
#include <cmath>
#include <cstdlib>

#include "b2_utils.h"
#include <iostream>

#define MAX_PLANTS 100
#define MAX_PARTICLES 1000

struct Particle {
    b2BodyId bodyId = b2_nullBodyId; // Box2D body
    float length;
    float thickness;
    Color color;
    int parentIndex;
    bool isGrowingTip;
    int level;
};

enum PlantStage {
    SEEDLING = 10,
    JUVENILE = 30,
    MATURE = 60,
    FLOWERING = 70,
    FRUITING = 80,
    DECAY
};

struct GrowthStageModifiers {
    float tipGrowthRateMultiplier;
    float branchProbabilityMultiplier;
    float tipAngleVarianceMultiplier;
    float lengthMultiplier;
    float thicknessMultiplier;
};

struct Plant {
    std::vector<Particle> particles;
    Color primaryColor;
    Color secondaryColor;
    int maxSize;
    int age;
    
    // Growth parameters
    float tipAngleVariance = 0.15f; // radians
    float branchingAngle = 0.6f;
    float branchAngleVariance = 0.05f;
    float branchProbability = 0.33f;
};

void DeletePlant(Plant& plant) {
    for (auto& particle : plant.particles) {
        if (b2Body_IsValid(particle.bodyId)) {
            b2DestroyBody(particle.bodyId);
            particle.bodyId = b2_nullBodyId;
        }
    }
    plant.particles.clear();
}

static PlantStage GetPlantStage(Plant& plant) {
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

GrowthStageModifiers GetGrowthModifiers(PlantStage stage) {
    switch (stage) {
        case SEEDLING:
            return { 1.5f, 0.5f, 0.7f, 1.2f, 0.9f }; // Taller, fewer branches, straighter
        case JUVENILE:
            return { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f }; // Baseline growth
        case MATURE:
            return { 0.8f, 1.5f, 1.2f, 0.9f, 0.9f }; // Slower tips, more branching
        case FLOWERING:
        case FRUITING:
            return { 0.0f, 0.0f, 0.0f, 1.0f, 1.0f }; // Growth stops, future: flowers/fruits
        default:
            return { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
    }
}

Particle CreateParticle(b2WorldId worldId, Vector2 pos, float angle, float length, float thickness, Color color, int parentIndex, const Particle* parent, bool isGrowingTip) {
    b2BodyDef bd = b2DefaultBodyDef();
    b2ShapeDef shapeDef = b2DefaultShapeDef();

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
        jointDef.hertz = length * 2 + thickness * 10;
        jointDef.dampingRatio = 0;//0.15f;
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
        isGrowingTip,
        level
    };

    return p;
}

float RandomFloat(float min, float max) {
    return min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
}

Vector2 RotateVector(Vector2 v, float angle) {
    float cs = cosf(angle);
    float sn = sinf(angle);
    return { v.x * cs - v.y * sn, v.x * sn + v.y * cs };
}

void GrowPlant(Plant& plant, b2WorldId worldId) {
    std::vector<Particle> newParticles;

    plant.age++;
    auto stage = GetPlantStage(plant);
    if (stage == PlantStage::DECAY) return;
    auto modifiers = GetGrowthModifiers(stage);

    for (size_t i = 0; i < plant.particles.size(); i++) {
        if (plant.particles.size() + newParticles.size() >= (size_t)plant.maxSize) return; 
        Particle& p = plant.particles[i];
        if (!p.isGrowingTip) continue;
        
        float lerpStep = 0.5f*1/p.thickness; // How much to move toward secondary color
        Color newColor = ColorLerp(p.color, plant.secondaryColor, lerpStep);

        float parentAngle = b2Rot_GetAngle(b2Body_GetRotation(p.bodyId));
        b2ShapeId shapes[1];
        b2Body_GetShapes(p.bodyId, shapes, 1);
        b2Capsule segmentCapsule = b2Shape_GetCapsule(shapes[0]);
        Vector2 parentPos = rayVec2(b2Body_GetWorldPoint(p.bodyId, segmentCapsule.center1));
        Vector2 newPos = rayVec2(b2Body_GetWorldPoint(p.bodyId, segmentCapsule.center2));

        // Update current tip
        float tipAngle = RandomFloat(-plant.tipAngleVariance, plant.tipAngleVariance);
        Particle newParticle = CreateParticle(worldId,
            newPos,
            tipAngle,
            p.length * RandomFloat(0.9f, 1.1f), 
            p.thickness * RandomFloat(0.85f,0.99f), 
            newColor, 
            (int)i,
            &p,
            true);

        newParticles.push_back(newParticle);
        p.isGrowingTip = false;

        // Random lateral branch
        if (RandomFloat(0.0f, 1.0f) < plant.branchProbability) {
            float branchAngle = (p.level%2 ? 1 : -1)*plant.branchingAngle + RandomFloat(-plant.branchAngleVariance, plant.branchAngleVariance);
            Particle branch = CreateParticle(worldId,
                parentPos,
                branchAngle,
                p.length * RandomFloat(0.9f, 1.1f),
                p.thickness * RandomFloat(0.75f,0.9f),
                newColor,
                (int)i,
                &p,
                true);
            newParticles.push_back(branch);
        }
    }

    plant.particles.insert(plant.particles.end(), newParticles.begin(), newParticles.end());
}

float HashFloat(int seed) {
    // Simple hash to float in [0, 1]
    seed = (seed << 13) ^ seed;
    return (1.0f - ((seed * (seed * seed * 15731 + 789221) + 1376312589) & 0x7fffffff) / 1073741824.0f);
}

void DrawPlant(const Plant& plant, float lineBoilAmplitude=0.25, float lineBoilSpeed=2, float boilThicknessFactor=0.6, float baseOffset=0.05, float phaseScaling=0.667) {
    double time = GetTime();

    for (size_t i = 0; i < plant.particles.size(); i++) {
        const Particle& p = plant.particles[i];
        b2ShapeId shapes[1];
        b2Body_GetShapes(p.bodyId, shapes, 1);
        b2Capsule segmentCapsule = b2Shape_GetCapsule(shapes[0]);
        Vector2 base = rayVec2(b2Body_GetWorldPoint(p.bodyId, segmentCapsule.center1));//-b2Vec2{baseOffset*p.length,0}));
        Vector2 tip = rayVec2(b2Body_GetWorldPoint(p.bodyId, segmentCapsule.center2));

        // Use hash-based pseudo-random phase from particle index
        double phase = (p.level) * PI/phaseScaling + HashFloat(b2StoreBodyId(plant.particles[0].bodyId))/4;

        //Vector2 startpos = Vector2Lerp(base, tip, sinf(time * 5 + phase + 1) * 0.1f - 0.1f);
        //Vector2 endpos = Vector2Lerp(base, tip, sinf(time * 5 + phase) * 0.1f + 0.9f);

        Vector2 center = Vector2Lerp(base, tip, 0.5f);
        float len = Vector2Length(Vector2Subtract(base, tip));
        Vector2 perp = {(base.y - tip.y) / len, -(base.x - tip.x) / len};

        float thickFactor = Clamp((plant.particles[0].thickness-p.thickness)/(boilThicknessFactor*plant.particles[0].thickness), 0, 1);
        float boilAmplitude = lineBoilAmplitude * len * thickFactor;
        float boilSpeed = lineBoilSpeed * thickFactor;
        float boilOffset = sinf(time * boilSpeed + phase) * boilAmplitude;

        Vector2 cppos = Vector2Add(center, Vector2Scale(perp, boilOffset));

        //DrawSplineSegmentBezierQuadratic(startpos, cppos, endpos, segmentCapsule.radius, p.color);
        DrawSplineSegmentBezierQuadratic(base, cppos, tip, segmentCapsule.radius*2, p.color);
        DrawCircle(base.x, base.y, segmentCapsule.radius*.9, p.color);
        DrawCircle(tip.x, tip.y, segmentCapsule.radius*.9, p.color);
    }
}

void WindowFunctions(int screenWidth, int screenHeight)
{
    // check for alt + enter
    if (IsKeyPressed(KEY_F) && IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL))
    {
        // see what display we are on right now
        int display = GetCurrentMonitor();

        if (IsWindowFullscreen())
        {
            // if we are full screen, then go back to the windowed size
            SetWindowSize(screenWidth, screenHeight);
        }
        else
        {
            // if we are not full screen, set the window size to match the monitor we are on
            SetWindowSize(GetMonitorWidth(display), GetMonitorHeight(display));
        }

        // toggle the state
        ToggleFullscreen();
    }
}

struct CastResult
{
	b2Vec2 point;
	b2Vec2 normal;
	b2BodyId bodyId;
	float fraction;
	bool hit;
};

static float CastCallback( b2ShapeId shapeId, b2Vec2 point, b2Vec2 normal, float fraction, void* context )
{
	CastResult* result = (CastResult*)context;
	result->point = point;
	result->normal = normal;
	result->bodyId = b2Shape_GetBody( shapeId );
	result->fraction = fraction;
	result->hit = true;
	return fraction;
}

b2BodyId GetBodyAtPoint(b2WorldId worldId, Vector2 point) {
    float queryRadius = 10.0f;

    b2QueryFilter filter = b2DefaultQueryFilter();
    b2ShapeProxy inputShapeProxy = {{point.x, point.y}, 1, queryRadius};
    CastResult castResult = {};
    b2World_CastShape(worldId, &inputShapeProxy, {0,0}, filter, CastCallback, &castResult);
    
    if (castResult.hit)
    {
        return castResult.bodyId;
    }
    else
    {
        return b2_nullBodyId;
    }
}

int main() {
    std::vector<Plant> plants;
    int tick = 0;
    const int GROWTH_TICK_RATE = 5;

    int screenWidth = 1200;
    int screenHeight = 675;

    //SetConfigFlags(FLAG_VSYNC_HINT);
    InitWindow(screenWidth, screenHeight, "2D Climbing Plant Growth (Raylib)");
    SetTargetFPS(120);
    float fixedDeltaTime = 1.0f/60;

    Camera2D camera = { 0 };
    camera.zoom = 1.0f;

	// 128 pixels per meter is a appropriate for this scene.
	const float lengthUnitsPerMeter = 64.0f*2;
	b2SetLengthUnitsPerMeter(lengthUnitsPerMeter);

	b2WorldDef worldDef = b2DefaultWorldDef();

	// Realistic gravity is achieved by multiplying gravity by the length unit.
	worldDef.gravity.y = 9.8f * lengthUnitsPerMeter;
	b2WorldId worldId = b2CreateWorld(&worldDef);

    // Plant particle dragging
	b2BodyId m_groundBodyId = b2_nullBodyId;
    b2BodyId selectedBodyId = b2_nullBodyId;
    b2JointId mouseJointId = b2_nullJointId;

    
    //float boilAmplitudeValue = 0.5f;
    //float boilSpeedValue = 2.0f;
    //float boilThicknessScaling = 0.5f;
    //float baseOffsetValue = 0.0f;
    //float phaseScalingValue = 2.0f;

    while (!WindowShouldClose()) {
        WindowFunctions(screenWidth, screenHeight);

        // Draw the sliders and update their values
        //GuiSliderBar({ 100, 60, 300, 20 }, TextFormat("boilAmplitude:%0.2f", boilAmplitudeValue), NULL, &boilAmplitudeValue, 0.0f, 5.0f);
        //GuiSliderBar({ 100, 90, 300, 20 },  TextFormat("boilSpeed:%0.2f", boilSpeedValue), NULL, &boilSpeedValue, 0.0f, 10.0f);
        //GuiSliderBar({ 100, 120, 300, 20 }, TextFormat("boilThicknessScaling:%0.2f", boilThicknessScaling), NULL, &boilThicknessScaling, 0.0f, 1.0f);
        //GuiSliderBar({ 100, 150, 300, 20 }, TextFormat("baseOffsetValue:%0.2f", baseOffsetValue), NULL, &baseOffsetValue, -1.0f, 1.0f);
        //GuiSliderBar({ 100, 180, 300, 20 }, TextFormat("phaseScaling:%0.2f", phaseScalingValue), NULL, &phaseScalingValue, 0.0f, 10.0f);

        // Handle Mouse Input
        Vector2 ssMousePos = GetMousePosition();
        DrawText(TextFormat("Mouse: (%0.1f,%0.1f)" ,ssMousePos.x, ssMousePos.y), 30, 30, 20, DARKGRAY);
        Vector2 mousePos = GetScreenToWorld2D(ssMousePos, camera);
        DrawText(TextFormat("World: (%0.1f,%0.1f)" ,mousePos.x, mousePos.y), 30, 50, 20, DARKGRAY);

        // CAMERA NAVIGATION
        float cameraSpeed = 500 * GetFrameTime(); // Pixels per second
        float edgeThreshold = 30.0f; // Distance from edge to start moving camera

        if (ssMousePos.x < edgeThreshold) camera.target.x -= cameraSpeed;
        if (ssMousePos.x > screenWidth - edgeThreshold) camera.target.x += cameraSpeed;
        if (ssMousePos.y < edgeThreshold) camera.target.y -= cameraSpeed;
        if (ssMousePos.y > screenHeight - edgeThreshold) camera.target.y += cameraSpeed;

        // DRAGGING PLANTS
        b2Vec2 mp = b2Vec2{ mousePos.x, mousePos.y };

        if (!b2Joint_IsValid(mouseJointId)) {
            mouseJointId = b2_nullJointId;
            selectedBodyId = b2_nullBodyId;
        } else if (B2_IS_NULL(mouseJointId) || B2_IS_NULL(selectedBodyId)) {
            b2DestroyJoint(mouseJointId);
            mouseJointId = b2_nullJointId;
            selectedBodyId = b2_nullBodyId;
        }
        // Start dragging
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
            b2BodyId bodyUnderCursor = GetBodyAtPoint(worldId, mousePos);
            if B2_IS_NON_NULL(bodyUnderCursor) {
                selectedBodyId = bodyUnderCursor;

                b2BodyDef bodyDef = b2DefaultBodyDef();
                m_groundBodyId = b2CreateBody(worldId, &bodyDef);

                b2MouseJointDef mouseJointDef = b2DefaultMouseJointDef();
                mouseJointDef.bodyIdA = m_groundBodyId;
                mouseJointDef.bodyIdB = selectedBodyId;
                mouseJointDef.target = mp;
                mouseJointDef.hertz = 7.5f;
                mouseJointDef.dampingRatio = 0.7f;
                mouseJointDef.collideConnected = true;
			    mouseJointDef.maxForce = 1000.0f * b2Body_GetMass(selectedBodyId) * b2Length(b2World_GetGravity(worldId));
                mouseJointId = b2CreateMouseJoint(worldId, &mouseJointDef);
            }
        } else if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON)) {
            // Stop dragging
            if B2_IS_NON_NULL(mouseJointId) {
                b2DestroyJoint(mouseJointId);
                mouseJointId = b2_nullJointId;
                selectedBodyId = b2_nullBodyId;
            }
        }

        // Dragging
        if (B2_IS_NON_NULL(mouseJointId)) {
            std::cout << "dragging(" << b2StoreBodyId(selectedBodyId) << "): [" << mousePos.x << ", " << mousePos.y << "]\n";
            b2MouseJoint_SetTarget(mouseJointId, mp);
		    b2Body_SetAwake( selectedBodyId, true );
        }

        // PLANTING SEEDS
        if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON) && plants.size() < MAX_PLANTS) {
            Plant plant;
            plant.maxSize = GetRandomValue(5, 100);
            plant.primaryColor = Color{static_cast<unsigned char>(GetRandomValue(0, 255)),
                static_cast<unsigned char>(GetRandomValue(0, 255)),
                static_cast<unsigned char>(GetRandomValue(0, 255)),
                255};
            plant.secondaryColor = Color{static_cast<unsigned char>(GetRandomValue(0, 255)),
                static_cast<unsigned char>(GetRandomValue(0, 255)),
                static_cast<unsigned char>(GetRandomValue(0, 255)),
                255};
            plant.branchingAngle = RandomFloat(0.1f, PI/2);
            plant.branchAngleVariance = RandomFloat(0.05f, 0.2f);
            plant.tipAngleVariance = RandomFloat(0.05f, 0.3f);
            Particle seed = CreateParticle(worldId,
                mousePos,
                RandomFloat(-PI/2.1, -PI/1.9),
                RandomFloat(10,40),
                RandomFloat(2,20),
                plant.primaryColor, // Start with plant’s base color
                -1,
                nullptr,
                true);
            plant.particles.push_back(seed);
            plants.push_back(plant);
        }

        // DELETING PLANTS
        if (IsKeyPressed(KEY_R) && plants.size() > 0)
        {
            for (Plant& plant : plants) {
                DeletePlant(plant);
            }
            plants.clear();
        }
        
        // Update
        tick++;
        if (tick % GROWTH_TICK_RATE == 0)
        {
            for (Plant& plant : plants) {
                GrowPlant(plant, worldId);
            }
        }

        // Physics Update
        float deltaTime = GetFrameTime();
        b2World_Step(worldId, fixedDeltaTime, 10);


        // DRAW
        ClearBackground(BLACK);
        BeginDrawing();

        BeginMode2D(camera);


        for (const Plant& plant : plants) {
            DrawPlant(plant);//, boilAmplitudeValue, boilSpeedValue, boilThicknessScaling, baseOffsetValue, phaseScalingValue);
        }

        EndMode2D();

        DrawText("Right-Click to plant a seed!", 10, 10, 10, DARKGRAY);
        DrawFPS(screenWidth-100, 10);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
