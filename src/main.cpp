#include "raylib.h"
#include "raymath.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#include "box2d/box2d.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "b2_utils.h"
#include "plants.h"
#include "random.h"

#define MAX_PLANTS 100
#define MAX_PARTICLES 1000

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

void DrawPlantInfoBox(const Plant& plant, Vector2 mousePos) {
    PlantStage stage = GetPlantStage((Plant&)plant);
    const char* stageName = "Unknown";
    switch (stage) {
        case PlantStage::SEEDLING: stageName = "Seedling"; break;
        case PlantStage::JUVENILE: stageName = "Juvenile"; break;
        case PlantStage::MATURE: stageName = "Mature"; break;
        case PlantStage::FLOWERING: stageName = "Flowering"; break;
        case PlantStage::FRUITING: stageName = "Fruiting"; break;
        case PlantStage::DECAY: stageName = "Decay"; break;
    }

    // Build plant info text
    std::string infoText = TextFormat(" \\
        Stage:%s\n \\
        Age:%d\n \\
        Max Size:%d\n \\
        Tip Angle Var:%0.2f\n \\
        Branching Angle:%0.2f\n \\
        Branch Angle Var:%0.2f\n \\
        Branch Probability:%0.2f\n \\
        Secondary Growth Rate:%0.2f\n",
        stageName,
        plant.age,
        plant.maxSize,
        plant.tipAngleVariance,
        plant.branchingAngle,
        plant.branchAngleVariance,
        plant.branchProbability,
        plant.secondaryGrowthRate
        );

    // Calculate box size
    Vector2 boxSize = { 210, 150 }; // Adjust based on your text amount
    Rectangle infoBox = { mousePos.x + 10, mousePos.y + 10, boxSize.x, boxSize.y };

    // Keep on screen
    if (infoBox.x + infoBox.width > GetScreenWidth()) infoBox.x -= (infoBox.width + 20);
    if (infoBox.y + infoBox.height > GetScreenHeight()) infoBox.y -= (infoBox.height + 20);

    auto linePadding = GuiGetStyle(DEFAULT, TEXT_LINE_SPACING);
    GuiSetStyle(DEFAULT, TEXT_LINE_SPACING, 14);
    // Draw using raygui
    GuiPanel(infoBox, "Plant Info");
    GuiLabel({infoBox.x, infoBox.y + 40, infoBox.width, infoBox.height - 40}, infoText.c_str());
    GuiSetStyle(DEFAULT, TEXT_LINE_SPACING, linePadding);
}

int main() {
    std::vector<Plant> plants;
    int tick = 0;
    //const int GROWTH_TICK_RATE = 10;
    float growthTickRate = 10;

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
    float deformationThreshold = 4 * DEG2RAD;
    float resistanceThreshold = 8 * DEG2RAD;
    float yieldThreshold = 25 * DEG2RAD;

    while (!WindowShouldClose()) {
        WindowFunctions(screenWidth, screenHeight);

        // Draw the sliders and update their values
        //GuiSliderBar({ 100, 60, 300, 20 }, TextFormat("boilAmplitude:%0.2f", boilAmplitudeValue), NULL, &boilAmplitudeValue, 0.0f, 5.0f);
        //GuiSliderBar({ 100, 90, 300, 20 },  TextFormat("boilSpeed:%0.2f", boilSpeedValue), NULL, &boilSpeedValue, 0.0f, 10.0f);
        //GuiSliderBar({ 100, 120, 300, 20 }, TextFormat("boilThicknessScaling:%0.2f", boilThicknessScaling), NULL, &boilThicknessScaling, 0.0f, 1.0f);
        //GuiSliderBar({ 100, 150, 300, 20 }, TextFormat("baseOffsetValue:%0.2f", baseOffsetValue), NULL, &baseOffsetValue, -1.0f, 1.0f);
        //GuiSliderBar({ 100, 180, 300, 20 }, TextFormat("phaseScaling:%0.2f", phaseScalingValue), NULL, &phaseScalingValue, 0.0f, 10.0f);
        GuiSliderBar({ 100, 120, 300, 20 }, TextFormat("deformationThreshold:%0.2f", deformationThreshold * RAD2DEG), NULL, &deformationThreshold, 0.0f, PI);
        GuiSliderBar({ 100, 150, 300, 20 }, TextFormat("resistanceThreshold:%0.2f", resistanceThreshold * RAD2DEG), NULL, &resistanceThreshold, 0.0f, PI);
        GuiSliderBar({ 100, 180, 300, 20 }, TextFormat("yieldThreshold:%0.2f", yieldThreshold * RAD2DEG), NULL, &yieldThreshold, 0.0f, PI);
        GuiSliderBar({ 100, 210, 300, 20 }, TextFormat("growthTickRate:%d", (int)growthTickRate), NULL, &growthTickRate, 1.0f, 120.0f);
        for (auto p : plants) 
        {
            p.deformationThreshold = deformationThreshold;
            p.resistanceThreshold = resistanceThreshold;
            p.yieldThreshold = yieldThreshold;
        }

        // Handle Mouse Input
        Vector2 ssMousePos = GetMousePosition();
        DrawText(TextFormat("Mouse: (%0.1f,%0.1f)" ,ssMousePos.x, ssMousePos.y), 30, 30, 20, DARKGRAY);
        Vector2 mousePos = GetScreenToWorld2D(ssMousePos, camera);
        DrawText(TextFormat("World: (%0.1f,%0.1f)" ,mousePos.x, mousePos.y), 30, 50, 20, DARKGRAY);

        // CAMERA NAVIGATION
        float cameraSpeed = 250 * GetFrameTime(); // Pixels per second
        float edgeThreshold = 15.0f; // Distance from edge to start moving camera

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
                mouseJointDef.hertz = 10.0f;
                mouseJointDef.dampingRatio = 1.0f;
                mouseJointDef.collideConnected = true;
			    mouseJointDef.maxForce = 100000.0f * b2Body_GetMass(selectedBodyId) * std::max(b2Length(b2World_GetGravity(worldId)), 1.0f);
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
		    b2Body_SetAwake( selectedBodyId, true );
            b2MouseJoint_SetTarget(mouseJointId, mp);
        }

        // PLANTING SEEDS
        if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON) && plants.size() < MAX_PLANTS) {
            Plant plant;
            plant.maxSize = GetRandomValue(5, 250);
            plant.primaryColor = Color{static_cast<unsigned char>(GetRandomValue(0, 255)),
                static_cast<unsigned char>(GetRandomValue(0, 255)),
                static_cast<unsigned char>(GetRandomValue(0, 255)),
                255};
            plant.secondaryColor = Color{static_cast<unsigned char>(GetRandomValue(0, 255)),
                static_cast<unsigned char>(GetRandomValue(0, 255)),
                static_cast<unsigned char>(GetRandomValue(0, 255)),
                255};
            plant.branchingAngle = RandomFloatRange(0.1f, PI/2);
            plant.branchAngleVariance = RandomFloatRange(0.05f, 0.2f);
            plant.tipAngleVariance = RandomFloatRange(0.05f, 0.3f);
            Particle seed = CreateParticle(worldId,
                mousePos,
                RandomFloatRange(-PI/2.1, -PI/1.9),
                RandomFloatRange(10,40),
                RandomFloatRange(2,20),
                plant.primaryColor, // Start with plant’s base color
                -1,
                nullptr,
                ParticleType::GROWING_TIP);
                
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
        if (tick % (int)growthTickRate == 0)
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
        

        // Plant Info
        Plant* hoveredPlant = GetPlantAtPoint(plants, worldId, mousePos);

        if (hoveredPlant != nullptr) {
            DrawPlantInfoBox(*hoveredPlant, mousePos);
        }

        EndMode2D();

        DrawText("Right-Click to plant a seed!", 10, 10, 10, DARKGRAY);
        DrawFPS(screenWidth-100, 10);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
