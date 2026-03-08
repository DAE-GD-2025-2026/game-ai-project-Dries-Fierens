// Fill out your copyright notice in the Description page of Project Settings.


#include "Level_Flocking.h"


// Sets default values
ALevel_Flocking::ALevel_Flocking()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALevel_Flocking::BeginPlay()
{
	Super::BeginPlay();

	TrimWorld->SetTrimWorldSize(2200.f);
	TrimWorld->bShouldTrimWorld = true;
	
	pAgentToEvade = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector::ZeroVector, FRotator::ZeroRotator);
	
	pEvadeTargetWander = std::make_unique<Wander>();
	pEvadeTargetWander->SetDebugColor(FColor::Red);
	pAgentToEvade->SetSteeringBehavior(pEvadeTargetWander.get());
	
	pFlock = TUniquePtr<Flock>(
		new Flock(
			GetWorld(),
			SteeringAgentClass,
			FlockSize,
			TrimWorld->GetTrimWorldSize(),
			pAgentToEvade,
			true)
			);
}

// Called every frame
void ALevel_Flocking::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	pFlock->ImGuiRender(WindowPos, WindowSize);
	pAgentToEvade->Tick(DeltaTime);
	pFlock->Tick(DeltaTime);
	pFlock->RenderDebug();
	if (bUseMouseTarget)
		pFlock->SetTarget_Seek(MouseTarget);
}

