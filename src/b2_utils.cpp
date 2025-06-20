#include "b2_utils.h"

#include "box2d/box2d.h"
#include <vector>

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
    float queryRadius = 15.0f;

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

struct RadiusQueryContext
{
    std::vector<b2BodyId>* foundBodies;
    int maxBodies;
};

static float BodiesInRadiusCallback(b2ShapeId shapeId, b2Vec2 point, b2Vec2 normal, float fraction, void* context)
{
    RadiusQueryContext* queryContext = (RadiusQueryContext*)context;

    // Stop if we've already found enough bodies
    if ((int)queryContext->foundBodies->size() >= queryContext->maxBodies)
    {
        return 0.0f; // Early out: stops the query
    }

    b2BodyId bodyId = b2Shape_GetBody(shapeId);

    // Avoid duplicates
    if (std::find_if(queryContext->foundBodies->begin(), queryContext->foundBodies->end(), 
        [&](b2BodyId id) { return B2_ID_EQUALS(id, bodyId); }) == queryContext->foundBodies->end())
    {
        queryContext->foundBodies->push_back(bodyId);
    }

    return 1.0f; // Continue the query
}

std::vector<b2BodyId> GetBodiesWithinRadius(b2WorldId worldId, b2Vec2 point, float queryRadius, int maxBodies)
{
    std::vector<b2BodyId> bodiesFound;
    bodiesFound.reserve(maxBodies);

    b2QueryFilter filter = b2DefaultQueryFilter();

    b2ShapeProxy inputShapeProxy = { {point}, 1, queryRadius };

    RadiusQueryContext queryContext;
    queryContext.foundBodies = &bodiesFound;
    queryContext.maxBodies = maxBodies;

    b2World_CastShape(worldId, &inputShapeProxy, { 0, 0 }, filter, BodiesInRadiusCallback, &queryContext);

    return bodiesFound;
}