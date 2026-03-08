#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};
	
	steering.LinearVelocity = Target.Position - Agent.GetPosition();
	//steering.LinearVelocity.Normalize(); // will be normalized by default
	
	if (Agent.GetDebugRenderingEnabled()) 
	{
		const FVector start = FVector(Agent.GetPosition(), 0);
		const FVector end = FVector(Target.Position, 0);
		DrawDebugDirectionalArrow(Agent.GetWorld(), start, end, 5.f, FColor::Red);
	}

	return steering;
}

SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};
	
	steering.LinearVelocity = Agent.GetPosition() - Target.Position;

	if (Agent.GetDebugRenderingEnabled()) 
	{
		const FVector start = FVector(Agent.GetPosition(), 0);
		const FVector end = FVector(Agent.GetPosition() + steering.LinearVelocity, 0);
		DrawDebugDirectionalArrow(Agent.GetWorld(), start, end, 5.f, FColor::Red);
	}
	
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
	
	if(Agent.GetDebugRenderingEnabled())
	{
		const FVector pos = FVector(Agent.GetPosition(), 0);
		DrawDebugCircle(Agent.GetWorld(), pos, m_SlowRadius,0.7f, FColor::Red);
		DrawDebugCircle(Agent.GetWorld(), pos, m_TargetRadius,0.7f, FColor::Red);
		const FVector end = FVector(Target.Position, 0);
		DrawDebugDirectionalArrow(Agent.GetWorld(), pos, end, 5.f, FColor::Red);
	}

	return steering;
}

SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};
	
	FVector2D toTarget = Target.Position - Agent.GetPosition();
	toTarget.Normalize();

	const float desiredAngle = atan2(toTarget.Y, toTarget.X);
	const float currentAngle = Agent.GetRotation();
	
	// Calculate the shortest angle to rotate
	float angleDifference = desiredAngle - currentAngle;
	if (angleDifference > PI) angleDifference -= 2 * PI;
	if (angleDifference < -PI) angleDifference += 2 * PI;

	steering.AngularVelocity = angleDifference;
	
	if (Agent.GetDebugRenderingEnabled()) 
	{
		const FVector start = FVector(Agent.GetPosition(), 0);
		const FVector end = FVector(Target.Position, 0);
		DrawDebugDirectionalArrow(Agent.GetWorld(), start, end, 5.f, FColor::Red);
	}

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
	
	if (Agent.GetDebugRenderingEnabled()) 
	{
		const FVector start = FVector(Agent.GetPosition(), 0);
		const FVector end = FVector(Agent.GetPosition() + steering.LinearVelocity, 0);
		DrawDebugDirectionalArrow(Agent.GetWorld(), start, end, 5.f, FColor::Red);
	}

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
	
	if (Agent.GetDebugRenderingEnabled()) 
	{
		const FVector start = FVector(Agent.GetPosition(), 0);
		const FVector end = FVector(Agent.GetPosition() + steering.LinearVelocity, 0);
		DrawDebugDirectionalArrow(Agent.GetWorld(), start, end, 5.f, FColor::Red);
	}

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

	if (Agent.GetDebugRenderingEnabled())
	{
		const FVector circlePos = FVector(Agent.GetPosition() + circleCenter, 0);
		DrawDebugCircle(Agent.GetWorld(), circlePos, 200.F, 100.f, FColor::Blue, false, -1, 0.f, 4.f, FVector::RightVector, FVector::ForwardVector);
		const FVector start = FVector(Agent.GetPosition(), 0);
		const FVector end = FVector(Agent.GetPosition() + steering.LinearVelocity, 0);
		DrawDebugDirectionalArrow(Agent.GetWorld(), start, end, 5.f, FColor::Red);
	}
	
	return steering;
}
