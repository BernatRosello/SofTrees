#pragma once
#include <cstdint>
#include "raylib.h"

#define RAND_LIMIT 32767
#define RAND_SEED 12345


extern uint32_t g_randomSeed;

// Simple random number generator. Using this instead of rand() for cross-platform determinism.
inline int RandomInt()
{
	// XorShift32 algorithm
	uint32_t x = g_randomSeed;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	g_randomSeed = x;

	// Map the 32-bit value to the range 0 to RAND_LIMIT
	return (int)( x % ( RAND_LIMIT + 1 ) );
}

// Random integer in range [lo, hi]
inline int RandomIntRange( int lo, int hi )
{
	return lo + RandomInt() % ( hi - lo + 1 );
}

// Random number in range [-1,1]
inline float RandomFloat()
{
	float r = (float)( RandomInt() & ( RAND_LIMIT ) );
	r /= RAND_LIMIT;
	r = 2.0f * r - 1.0f;
	return r;
}

// Random floating point number in range [lo, hi]
inline float RandomFloatRange( float lo, float hi )
{
	float r = (float)( RandomInt() & ( RAND_LIMIT ) );
	r /= RAND_LIMIT;
	r = ( hi - lo ) * r + lo;
	return r;
}

// Random vector with coordinates in range [lo, hi]
inline Vector2 RandomVec2( float lo, float hi )
{
	Vector2 v;
	v.x = RandomFloatRange( lo, hi );
	v.y = RandomFloatRange( lo, hi );
	return v;
}