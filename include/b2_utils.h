#pragma once

#include "box2d/box2d.h"
#include "raylib.h"
#include <vector>

inline Vector2 rayVec2(b2Vec2 v) {return Vector2{v.x,v.y};}
b2BodyId GetBodyAtPoint(b2WorldId worldId, Vector2 point);
std::vector<b2BodyId> GetBodiesWithinRadius(b2WorldId worldId, b2Vec2 point, float queryRadius, int maxBodies = 10);
