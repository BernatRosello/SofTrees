#pragma once

#include "raymath.h"
#include "box2d/box2d.h"

inline Vector2 rayVec2(b2Vec2 v) {return {v.x,v.y};}