#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"


//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	FVector2D avgPosition = pFlock->GetAverageNeighborPos();
	SetTarget(FTargetData(avgPosition));
	return Seek::CalculateSteering(deltaT, pAgent);
}

//*********************
//SEPARATION (FLOCKING)
SteeringOutput Separation::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	FVector2D separationForce = FVector2D{0, 0};
	const auto& neighbors = pFlock->GetNeighbors();
	for (const auto& neighbor : neighbors.GetPool())
	{
		if (neighbor) 
		{
			if (neighbor != &pAgent)
			{
				FVector2D toAgent = pAgent.GetPosition() - neighbor->GetPosition();
				float distance = toAgent.Size(); // magnitude
				if (distance > 0)
				{
					separationForce += toAgent / (distance * distance);
				}
			}
		}
	}
	separationForce.Normalize();
	separationForce *= pAgent.GetMaxLinearSpeed();

	SteeringOutput output;
	output.LinearVelocity = separationForce;
	output.AngularVelocity = 0;
	return output;
}

//*************************
//VELOCITY MATCH (FLOCKING)
SteeringOutput VelocityMatch::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	FVector2D averageVelocity = pFlock->GetAverageNeighborVelocity();
	averageVelocity.Normalize();
	averageVelocity *= pAgent.GetMaxLinearSpeed();

	SteeringOutput output;
	output.LinearVelocity = averageVelocity;
	return output;
}