// Fill out your copyright notice in the Description page of Project Settings.

#include "Level_FSM.h"

#include "FSMComponent.h"
#include "DecisionMaking/GameAIController.h"

namespace
{
	class FTestState final : public GameAI::FSM::State
	{
	public:
		FTestState()
			: State(TEXT("TestState"))
		{
		}

		virtual void Enter() override
		{
			UE_LOG(LogTemp, Log, TEXT("FSM entered TestState"));
		}

		virtual void Tick(float DeltaTime) override
		{
			if (UFSMComponent* FSMComponent = GetOwnerComponent())
			{
				if (AGameAIController* AIController = Cast<AGameAIController>(FSMComponent->GetOwner()))
				{
					if (APawn* ControlledPawn = AIController->GetPawn())
					{
						FRotator NewRotation = ControlledPawn->GetActorRotation();
						NewRotation.Yaw += 45.f * DeltaTime;
						ControlledPawn->SetActorRotation(NewRotation);
					}
				}
			}
		}

		virtual void Exit() override
		{
			UE_LOG(LogTemp, Log, TEXT("FSM exited TestState"));
		}
	};
}

// Sets default values
ALevel_FSM::ALevel_FSM()
{
	// Set this actor to call Tick() every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALevel_FSM::BeginPlay()
{
	Super::BeginPlay();

	if (!ensure(GetWorld() != nullptr))
	{
		return;
	}

	Agent = GetWorld()->SpawnActor<ASteeringAgent>(
		SteeringAgentClass,
		FVector{0.f, 0.f, 90.f},
		FRotator::ZeroRotator);

	if (!ensure(Agent != nullptr))
	{
		return;
	}

	Agent->SetDebugRenderingEnabled(false);

	if (Agent->GetController() == nullptr)
	{
		Agent->SpawnDefaultController();
	}

	if (AGameAIController* AIController = Cast<AGameAIController>(Agent->GetController()))
	{
		if (UFSMComponent* FSM = Cast<UFSMComponent>(AIController->GetBrainComponent()))
		{
			auto TestState = std::make_unique<FTestState>();
			GameAI::FSM::State* TestStatePtr = TestState.get();

			FSM->AddState(std::move(TestState));
			FSM->SetInitialState(TestStatePtr);

			AIController->RunFiniteStateMachine();
		}
	}
}

// Called every frame
void ALevel_FSM::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}