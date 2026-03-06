#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Shared/ImGuiHelpers.h"


Flock::Flock(
	UWorld* pWorld,
	TSubclassOf<ASteeringAgent> AgentClass,
	int FlockSize,
	float WorldSize,
	ASteeringAgent* const pAgentToEvade,
	bool bTrimWorld)
	: pWorld{pWorld}
	, FlockSize{ FlockSize }
	, pAgentToEvade{pAgentToEvade}
    , TrimWorld{bTrimWorld}
    , WorldSize{WorldSize}
    , pNeighbors{ FlockSize, NrOfNeighbors }
{
	Agents.SetNum(FlockSize);
	
	// TODO: initialize the flock and the memory pool
	pSeparationBehavior = std::make_unique<Separation>(this);
	pCohesionBehavior = std::make_unique<Cohesion>(this);
	pVelMatchBehavior = std::make_unique<VelocityMatch>(this);
	pSeekBehavior = std::make_unique<Seek>();
	pWanderBehavior = std::make_unique<Wander>();
	pEvadeBehavior = std::make_unique<Evade>(); //NeighborhoodRadius
	
	pBlendedSteering = std::make_unique<BlendedSteering>(
		std::vector<BlendedSteering::WeightedBehavior>({
			{pSeparationBehavior.get(), .8f},
			{pCohesionBehavior.get(), .6f},
			{pVelMatchBehavior.get(), .4f},
			{pSeekBehavior.get(), .3f},
			{pWanderBehavior.get(), .1f}
		}));
	
	pPrioritySteering = std::make_unique<PrioritySteering>(
		std::vector<ISteeringBehavior*>({
			pEvadeBehavior.get(),
			pBlendedSteering.get()
		}));
	
	pCellSpace = new CellSpace(pWorld, WorldSize, WorldSize, 10, 10, FlockSize);

	for (int i = 0; i < FlockSize; ++i)
	{
		Agents[i]->GetWorld()->SpawnActor<ASteeringAgent>(AgentClass, FVector{0,0,90}, FRotator::ZeroRotator);
		Agents[i]->SetIsAutoOrienting(true);
		Agents[i]->SetMaxLinearSpeed(15.f);
		Agents[i]->SetMass(1.f);
		Agents[i]->SetActorLocation(FVector{ FMath::FRandRange(0.f, WorldSize), FMath::FRandRange(0.f, WorldSize), 90 });
		Agents[i]->SetSteeringBehavior(pPrioritySteering.get());

		pCellSpace->AddAgent(*Agents[i]);
	}
}

Flock::~Flock()
{
 // TODO: Cleanup any additional data
}

void Flock::Tick(float DeltaTime)
{
  // TODO: update the flock
  // TODO: for every agent:
  for (ASteeringAgent* pAgent : Agents)
  {
  	pCellSpace->UpdateAgentCell(*pAgent, pAgent->GetPosition());
  	
  	// TODO: register the neighbors for this agent (-> fill the memory pool with the neighbors for the currently evaluated agent)
  	RegisterNeighbors(pAgent);
  	
  	FTargetData targetData = FTargetData{};
  	targetData.Position = pAgentToEvade->GetPosition();
  	targetData.LinearVelocity = pAgentToEvade->GetLinearVelocity();
  	pEvadeBehavior->SetTarget(targetData);
  	
  	// TODO: update the agent (-> the steeringbehaviors use the neighbors in the memory pool)
  	pAgent->Tick(DeltaTime);
  	// TODO: trim the agent to the world
  	if (TrimWorld)
  	{
  		
  	}
  }
}

void Flock::RenderDebug()
{
 // TODO: Render all the agents in the flock
	for (ASteeringAgent* pAgent : Agents)
		
	if (DebugRenderPartitions)
		pCellSpace->RenderCells();

	if (DebugRenderNeighborhood)
		RenderNeighborhood();
}

void Flock::ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize)
{
#ifdef PLATFORM_WINDOWS
#pragma region UI
	//UI
	{
		//Setup
		bool bWindowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", &bWindowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

		//Elements
		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: place target");
		ImGui::Text("RMB: move cam.");
		ImGui::Text("Scrollwheel: zoom cam.");
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("Flocking");
		ImGui::Spacing();

        // TODO: implement ImGUI checkboxes for debug rendering here
		ImGui::Checkbox("Render Neighborhood", &DebugRenderNeighborhood);
		ImGui::Checkbox("Render Partitions", &DebugRenderPartitions);
		
		ImGui::Text("Behavior Weights");
		ImGui::Spacing();

		// TODO: implement ImGUI sliders for steering behavior weights here
		ImGui::SliderFloat("Separation", &pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight, 0.f, 1.f, "%.2");
		ImGui::SliderFloat("Cohesion", &pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight, 0.f, 1.f, "%.2");
		ImGui::SliderFloat("Velocity Match", &pBlendedSteering->GetWeightedBehaviorsRef()[2].Weight, 0.f, 1.f, "%.2");
		ImGui::SliderFloat("Seek", &pBlendedSteering->GetWeightedBehaviorsRef()[3].Weight, 0.f, 1.f, "%.2");
		ImGui::SliderFloat("Wander", &pBlendedSteering->GetWeightedBehaviorsRef()[4].Weight, 0.f, 1.f, "%.2");
		
		//End
		ImGui::End();
	}
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
 // TODO: Debugrender the neighbors for the first agent in the flock
	if (Agents.IsEmpty()) return;

	// Register the neighbors for the first agent in the flock
	RegisterNeighbors(Agents[0]);
	FVector pos = FVector(Agents[0]->GetPosition(), 0);
	FVector dir = FVector(Agents[0]->GetPosition() + Agents[0]->GetLinearVelocity(), 0);
	DrawDebugCircle(Agents[0]->GetWorld(), pos, NeighborhoodRadius, 100.f, FColor::Blue);
	DrawDebugDirectionalArrow(Agents[0]->GetWorld(), pos, dir, 5.f, FColor::Red);

	// Debug render the neighbors
	for (int i = 0; i < NrOfNeighbors; ++i)
	{
		ASteeringAgent* neighbor = pNeighbors.GetPool()[i];
		FVector neighborPos = FVector(neighbor->GetPosition(), 0);
		DrawDebugCircle(neighbor->GetWorld(), neighborPos, NeighborhoodRadius, 100.f, FColor::Green);
	}
}

#ifndef GAMEAI_USE_SPACE_PARTITIONING
void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
{
	if (DebugRenderPartitions)
	{
		pCellSpace->RegisterNeighbors(*pAgent, NeighborhoodRadius);
		NrOfNeighbors = pCellSpace->GetNrOfNeighbors();
		return;
	}

	pNeighbors.reset();

	FVector2D agentPos = pAgent->GetPosition();

	for (ASteeringAgent* pOtherAgent : Agents)
	{
		if (pOtherAgent == pAgent)
			continue;

		float distanceSquared = (agentPos - pOtherAgent->GetPosition()).SizeSquared();

		if (distanceSquared <= NeighborhoodRadius * NeighborhoodRadius)
		{
			ASteeringAgent** neighborSlot = pNeighbors.allocate();
			if (neighborSlot)
			{
				*neighborSlot = pOtherAgent;
			}
		}
	}
}
#endif

FVector2D Flock::GetAverageNeighborPos() const
{
	FVector2D avgPosition = FVector2D::ZeroVector;

	if (NrOfNeighbors == 0)
		return avgPosition;

	for (ASteeringAgent* neighbor : pNeighbors.GetPool())
	{
		if (neighbor) 
		{
			avgPosition += neighbor->GetPosition();
		}
	}

	avgPosition /= static_cast<float>(NrOfNeighbors);
	
	return avgPosition;
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	FVector2D avgVelocity = FVector2D::ZeroVector;

	if (NrOfNeighbors == 0)
		return avgVelocity;

	for (ASteeringAgent* neighbor : pNeighbors.GetPool())
	{
		if (neighbor) 
		{
			avgVelocity += neighbor->GetLinearVelocity();
		}
	}

	avgVelocity /= static_cast<float>(NrOfNeighbors);

	return avgVelocity;
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
	if (pSeekBehavior)
	{
		pSeekBehavior->SetTarget(Target);
	}
}

