#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};
	
	steering.LinearVelocity = Target.Position - Agent.GetPosition();
	//steering.LinearVelocity.Normalize(); // will be normalized by default

	if (Agent.GetDebugRenderingEnabled()) 
	{
		//DrawDebugCircle()
	}

	return steering;
}

SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};

	steering.LinearVelocity = Agent.GetPosition() - Target.Position;

	// add debug rendering for grades!!
	
	return steering;
}

SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};
	
	steering.LinearVelocity = Target.Position - Agent.GetPosition();
	const float distance = steering.LinearVelocity.Size() - m_TargetRadius;
	
	if (distance < m_SlowRadius)
	{
		float speed = m_MaxLinearSpeed * (distance / (m_SlowRadius + m_TargetRadius));
		Agent.SetMaxLinearSpeed(speed);
		steering.LinearVelocity *= speed;
	}
	else
	{
		Agent.SetMaxLinearSpeed(m_MaxLinearSpeed);
		steering.LinearVelocity *= m_MaxLinearSpeed;
	}
	
	// add debug rendering for grades!!

	return steering;
}

SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};

	steering.LinearVelocity = Target.Position - Agent.GetPosition();

	//if (Agent.GetDebugRenderingEnabled())

	return steering;
}

SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};
	
	FVector2D toTarget = Target.Position - Agent.GetPosition();
	const float distance = toTarget.Size(); // Gives magnitude

	const float speed = Agent.GetMaxLinearSpeed();
	const float predictionTime = distance / speed;

	FVector2D futurePosition = Target.Position + Target.LinearVelocity * predictionTime;

	steering.LinearVelocity = futurePosition - Agent.GetPosition();
	steering.LinearVelocity *= speed;

	return steering;  
}

SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};

	const FVector2D toTarget = Target.Position - Agent.GetPosition();
	const float distance = toTarget.Size();
	
	const float speed = Agent.GetMaxLinearSpeed();
	const float predictionTime = distance / speed;
	
	const FVector2D futurePosition = Target.Position + Target.LinearVelocity * predictionTime;

	steering.LinearVelocity = Agent.GetPosition() - futurePosition; // opposite of pursuit
	steering.LinearVelocity *= speed;

	return steering;
}

SteeringOutput Wander::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering = {};

	// Calculate the new wander angle
	m_WanderAngle += FMath::FRandRange(-m_MaxAngleChange, m_MaxAngleChange);
	
	// Calculate the circle center in front of the agent
	FVector2D circleCenter = Agent.GetLinearVelocity().GetSafeNormal(); // Get the direction
	circleCenter *= m_OffsetDistance;
	
	// Calculate the displacement vector
	FVector2D displacement = { cos(m_WanderAngle) * m_Radius, sin(m_WanderAngle) * m_Radius };
	
	// Calculate the wander target
	FVector2D wanderTarget = Agent.GetPosition() + circleCenter + displacement;
	
	// Seek towards the wander target
	steering.LinearVelocity = wanderTarget - Agent.GetPosition();
	steering.LinearVelocity *= Agent.GetMaxLinearSpeed();

	return steering;
}
