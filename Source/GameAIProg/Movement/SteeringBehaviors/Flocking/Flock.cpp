#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Shared/ImGuiHelpers.h"
#include "../SpacePartitioning/SpacePartitioning.h"

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
    , bTrimWorld{bTrimWorld}
    , WorldSize{WorldSize}
    , pNeighbors{ FlockSize, NrOfNeighbors }
{
	Agents.resize(FlockSize);
	OldPositions.SetNum(FlockSize);
	
	// TODO: initialize the flock and the memory pool
	pSeparationBehavior = std::make_unique<Separation>(this);
	pCohesionBehavior = std::make_unique<Cohesion>(this);
	pVelMatchBehavior = std::make_unique<VelocityMatch>(this);
	pSeekBehavior = std::make_unique<Seek>();
	pWanderBehavior = std::make_unique<Wander>();
	pEvadeBehavior = std::make_unique<Evade>(NeighborhoodRadius);
	
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
	
	pCellSpace = new CellSpace(pWorld, WorldSize, WorldSize, NrOfCellsX, NrOfCellsX, FlockSize);

	FActorSpawnParameters spawnParams{};
	spawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
	
	for (int i = 0; i < FlockSize; ++i)
	{
		FVector randomSpawn = FVector{ FMath::FRandRange(0.f, WorldSize), FMath::FRandRange(0.f, WorldSize), 90 };
		ASteeringAgent* agent = pWorld->SpawnActor<ASteeringAgent>(AgentClass, randomSpawn, FRotator::ZeroRotator, spawnParams);
		
		if (agent)
		{
			agent->SetActorTickEnabled(false); // prevents automatic Tick()
			//agent->PrimaryActorTick.bCanEverTick = false;
		}
		
		Agents[i] = agent;
		OldPositions[i] = agent->GetPosition();
		
		Agents[i]->SetSteeringBehavior(pPrioritySteering.get());

		pCellSpace->AddAgent(*Agents[i]);
	}
}

Flock::~Flock()
{
	// TODO: Cleanup any additional data
	if (pCellSpace) { delete (pCellSpace); (pCellSpace) = nullptr; }
	
	if (pAgentToEvade) pAgentToEvade->Destroy();
	
	for(ASteeringAgent* pAgent : Agents)
	{
		if (pAgent) pAgent->Destroy();
	}
	Agents.clear();
}

void Flock::Tick(float DeltaTime)
{
  // TODO: update the flock
  // TODO: for every agent:
  for (int i = 0; i < Agents.size(); ++i)
  {
  	ASteeringAgent* pAgent = Agents[i];
  	
  	if (pCellSpace) pCellSpace->UpdateAgentCell(*pAgent, OldPositions[i]);
  	OldPositions[i] = pAgent->GetPosition();
  	
  	// TODO: register the neighbors for this agent (-> fill the memory pool with the neighbors for the currently evaluated agent)
  	RegisterNeighbors(pAgent);
  	
  	FTargetData targetData = FTargetData{};
  	targetData.Position = pAgentToEvade->GetPosition();
  	targetData.LinearVelocity = pAgentToEvade->GetLinearVelocity();
  	pEvadeBehavior->SetTarget(targetData);
  	
  	// TODO: update the agent (-> the steeringbehaviors use the neighbors in the memory pool)
  	pAgent->Tick(DeltaTime);
  	
  	// TODO: trim the agent to the world
  	
  }
}

void Flock::RenderDebug()
{
 // TODO: Render all the agents in the flock
	//for (ASteeringAgent* pAgent : Agents)	
	
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
		bool renderNeighborhood = DebugRenderNeighborhood;
		if (ImGui::Checkbox("Render Neighborhood", &renderNeighborhood))
			DebugRenderNeighborhood = renderNeighborhood;

		bool useSpacePartitioning = bUseSpacePartitioning;
		if (ImGui::Checkbox("Use Spatial Partitioning", &useSpacePartitioning))
			bUseSpacePartitioning = useSpacePartitioning;

		bool renderPartitions = DebugRenderPartitions;
		if (ImGui::Checkbox("Render Partitions", &renderPartitions))
			DebugRenderPartitions = renderPartitions;
		
		ImGui::Text("Behavior Weights");
		ImGui::Spacing();

		// TODO: implement ImGUI sliders for steering behavior weights here
		if (pBlendedSteering)
		{
			auto& weighted = pBlendedSteering->GetWeightedBehaviorsRef();
			
			float w0 = weighted[0].Weight;
			if (ImGui::SliderFloat("Separation", &w0, 0.f, 1.f, "%.2f"))
				weighted[0].Weight = w0;
			float w1 = weighted[1].Weight;
			if (ImGui::SliderFloat("Cohesion", &w1, 0.f, 1.f, "%.2f"))
				weighted[1].Weight = w1;
			float w2 = weighted[2].Weight;
			if (ImGui::SliderFloat("Velocity Match", &w2, 0.f, 1.f, "%.2f"))
				weighted[2].Weight = w2;
			float w3 = weighted[3].Weight;
			if (ImGui::SliderFloat("Seek", &w3, 0.f, 1.f, "%.2f"))
				weighted[3].Weight = w3;
			float w4 = weighted[4].Weight;
			if (ImGui::SliderFloat("Wander", &w4, 0.f, 1.f, "%.2f"))
				weighted[4].Weight = w4;
		}
		
		//End
		ImGui::End();
	}
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
	// TODO: Debugrender the neighbors for the first agent in the flock
	if (Agents.empty()) return;

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

void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
{
	if (bUseSpacePartitioning)
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

int Flock::GetNrOfNeighbors() const
{
	if (bUseSpacePartitioning) return pCellSpace->GetNrOfNeighbors();
	return NrOfNeighbors;
}

MemoryPool<ASteeringAgent*>& Flock::GetNeighbors()
{
	if (bUseSpacePartitioning) {
		pNeighbors.reset();
		pNeighbors.SetPool(pCellSpace->GetNeighbors());
	}
	return pNeighbors;
}

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

