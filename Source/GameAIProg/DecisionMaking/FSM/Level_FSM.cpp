// Fill out your copyright notice in the Description page of Project Settings.

#include "Level_FSM.h"

#include "BehaviorTree/BlackboardComponent.h"
#include "DrawDebugHelpers.h"
#include "FSMComponent.h"
#include "InputAction.h"
#include "DecisionMaking/GameAIController.h"

namespace
{
	const FName TargetActorKey{TEXT("TargetActor")};
	const FName LastKnownTargetLocationKey{TEXT("LastKnownTargetLocation")};
	const FName SearchStartTimeKey{TEXT("SearchStartTime")};

	FVector2D To2D(const FVector& Value)
	{
		return FVector2D{Value.X, Value.Y};
	}

	bool IsTargetVisible(AGameAIController* GuardController, ASteeringAgent* Guard, ASteeringAgent* Thief, float DetectionRadius)
	{
		if (GuardController == nullptr || Guard == nullptr || Thief == nullptr)
		{
			return false;
		}

		const bool bInRange =
			FVector::DistSquared2D(Guard->GetActorLocation(), Thief->GetActorLocation()) <=
			FMath::Square(DetectionRadius);

		if (!bInRange)
		{
			return false;
		}

		return GuardController->LineOfSightTo(Thief);
	}

	void UpdateTargetBlackboard(UBlackboardComponent* Blackboard, ASteeringAgent* Thief)
	{
		if (Blackboard == nullptr || Thief == nullptr)
		{
			return;
		}

		Blackboard->SetValueAsObject(TargetActorKey, Thief);
		Blackboard->SetValueAsVector(LastKnownTargetLocationKey, Thief->GetActorLocation());
	}

	class FPatrolState final : public GameAI::FSM::State
	{
	public:
		FPatrolState(ASteeringAgent* InGuard, const TArray<FVector>& InPatrolRoute, float InAcceptanceRadius)
			: State(TEXT("Patrol"))
			, Guard(InGuard)
			, PatrolRoute(InPatrolRoute)
			, AcceptanceRadiusSq(FMath::Square(InAcceptanceRadius))
		{
		}

		virtual void Enter() override
		{
			if (Guard != nullptr)
			{
				Guard->SetSteeringBehavior(&SeekBehavior);
			}
		}

		virtual void Tick(float DeltaTime) override
		{
			if (Guard == nullptr || PatrolRoute.IsEmpty())
			{
				return;
			}

			if (CurrentPatrolIndex >= PatrolRoute.Num())
			{
				CurrentPatrolIndex = 0;
			}

			if (FVector::DistSquared2D(Guard->GetActorLocation(), PatrolRoute[CurrentPatrolIndex]) <= AcceptanceRadiusSq)
			{
				CurrentPatrolIndex = (CurrentPatrolIndex + 1) % PatrolRoute.Num();
			}

			SeekBehavior.SetTarget(FTargetData{To2D(PatrolRoute[CurrentPatrolIndex])});
			Guard->SetSteeringBehavior(&SeekBehavior);
		}

	private:
		ASteeringAgent* Guard{nullptr};
		TArray<FVector> PatrolRoute{};
		int32 CurrentPatrolIndex{0};
		float AcceptanceRadiusSq{0.f};
		Seek SeekBehavior{};
	};

	class FChaseState final : public GameAI::FSM::State
	{
	public:
		FChaseState(ASteeringAgent* InGuard, ASteeringAgent* InThief)
			: State(TEXT("Chase"))
			, Guard(InGuard)
			, Thief(InThief)
		{
		}

		virtual void Enter() override
		{
			if (Guard != nullptr)
			{
				Guard->SetSteeringBehavior(&PursuitBehavior);
			}
		}

		virtual void Tick(float DeltaTime) override
		{
			if (Guard == nullptr || Thief == nullptr)
			{
				return;
			}

			FTargetData TargetData{};
			TargetData.Position = Thief->GetPosition();
			TargetData.Orientation = Thief->GetRotation();
			TargetData.LinearVelocity = Thief->GetLinearVelocity();
			TargetData.AngularVelocity = Thief->GetAngularVelocity();

			PursuitBehavior.SetTarget(TargetData);
			Guard->SetSteeringBehavior(&PursuitBehavior);

			UpdateTargetBlackboard(GetBlackboard(), Thief);
		}

	private:
		ASteeringAgent* Guard{nullptr};
		ASteeringAgent* Thief{nullptr};
		Pursuit PursuitBehavior{};
	};

	class FSearchState final : public GameAI::FSM::State
	{
	public:
		FSearchState(ASteeringAgent* InGuard, float InArrivalRadius)
			: State(TEXT("Search"))
			, Guard(InGuard)
			, ArrivalRadiusSq(FMath::Square(InArrivalRadius))
		{
			ArriveBehavior.SetTargetRadius(InArrivalRadius);
			ArriveBehavior.SetSlowRadius(InArrivalRadius * 3.f);
		}

		virtual void Enter() override
		{
			bReachedLastKnownLocation = false;

			if (Guard != nullptr)
			{
				Guard->SetSteeringBehavior(&ArriveBehavior);

				if (UBlackboardComponent* BlackboardComp = GetBlackboard())
				{
					BlackboardComp->SetValueAsFloat(SearchStartTimeKey, Guard->GetWorld()->GetTimeSeconds());
				}
			}
		}

		virtual void Tick(float DeltaTime) override
		{
			if (Guard == nullptr)
			{
				return;
			}

			UBlackboardComponent* BlackboardComp = GetBlackboard();
			if (BlackboardComp == nullptr)
			{
				return;
			}

			const FVector LastKnownTargetLocation = BlackboardComp->GetValueAsVector(LastKnownTargetLocationKey);

			if (!bReachedLastKnownLocation)
			{
				ArriveBehavior.SetTarget(FTargetData{To2D(LastKnownTargetLocation)});
				Guard->SetSteeringBehavior(&ArriveBehavior);

				if (FVector::DistSquared2D(Guard->GetActorLocation(), LastKnownTargetLocation) <= ArrivalRadiusSq)
				{
					bReachedLastKnownLocation = true;
					Guard->SetSteeringBehavior(&WanderBehavior);
				}

				return;
			}

			Guard->SetSteeringBehavior(&WanderBehavior);
		}

	private:
		ASteeringAgent* Guard{nullptr};
		float ArrivalRadiusSq{0.f};
		bool bReachedLastKnownLocation{false};
		Arrive ArriveBehavior{};
		Wander WanderBehavior{};
	};
}

// Sets default values
ALevel_FSM::ALevel_FSM()
{
	PrimaryActorTick.bCanEverTick = true;

	PatrolRoute = {
		FVector{300.f, 300.f, 90.f},
		FVector{300.f, -300.f, 90.f},
		FVector{-300.f, -300.f, 90.f},
		FVector{-300.f, 300.f, 90.f}
	};
}

// Called when the game starts or when spawned
void ALevel_FSM::BeginPlay()
{
	Super::BeginPlay();

	if (!ensure(GetWorld() != nullptr) || !ensure(SteeringAgentClass != nullptr))
	{
		return;
	}

	const FVector GuardSpawnLocation = PatrolRoute.IsEmpty() ? FVector{300.f, 300.f, 90.f} : PatrolRoute[0];
	const FVector ThiefSpawnLocation = FVector{0.f, 0.f, 90.f};

	Thief = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, ThiefSpawnLocation, FRotator::ZeroRotator);
	Guard = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, GuardSpawnLocation, FRotator::ZeroRotator);

	if (!ensure(Thief != nullptr) || !ensure(Guard != nullptr))
	{
		return;
	}

	Thief->SetDebugRenderingEnabled(false);
	Guard->SetDebugRenderingEnabled(false);

	ThiefMoveBehavior.SetTargetRadius(10.f);
	ThiefMoveBehavior.SetSlowRadius(150.f);
	ThiefMoveBehavior.SetTarget(FTargetData{Thief->GetPosition()});
	Thief->SetSteeringBehavior(&ThiefMoveBehavior);

	if (Guard->GetController() == nullptr)
	{
		Guard->SpawnDefaultController();
	}

	AGameAIController* GuardController = Cast<AGameAIController>(Guard->GetController());
	if (!ensure(GuardController != nullptr))
	{
		return;
	}

	UFSMComponent* FSM = Cast<UFSMComponent>(GuardController->GetBrainComponent());
	if (!ensure(FSM != nullptr))
	{
		return;
	}

	auto PatrolState = std::make_unique<FPatrolState>(Guard, PatrolRoute, PatrolPointAcceptanceRadius);
	auto ChaseState = std::make_unique<FChaseState>(Guard, Thief);
	auto SearchState = std::make_unique<FSearchState>(Guard, SearchArrivalRadius);

	GameAI::FSM::State* PatrolStatePtr = PatrolState.get();
	GameAI::FSM::State* ChaseStatePtr = ChaseState.get();
	GameAI::FSM::State* SearchStatePtr = SearchState.get();

	FSM->AddState(std::move(PatrolState));
	FSM->AddState(std::move(ChaseState));
	FSM->AddState(std::move(SearchState));
	FSM->SetInitialState(PatrolStatePtr);

	FSM->AddTransition(PatrolStatePtr, ChaseStatePtr, [this, GuardController]()
	{
		const bool bVisible = IsTargetVisible(GuardController, Guard, Thief, DetectionRadius);
		if (bVisible)
		{
			UpdateTargetBlackboard(GuardController->GetBlackboardComponent(), Thief);
		}
		return bVisible;
	});

	FSM->AddTransition(ChaseStatePtr, SearchStatePtr, [this, GuardController]()
	{
		return !IsTargetVisible(GuardController, Guard, Thief, DetectionRadius);
	});

	FSM->AddTransition(SearchStatePtr, ChaseStatePtr, [this, GuardController]()
	{
		const bool bVisible = IsTargetVisible(GuardController, Guard, Thief, DetectionRadius);
		if (bVisible)
		{
			UpdateTargetBlackboard(GuardController->GetBlackboardComponent(), Thief);
		}
		return bVisible;
	});

	FSM->AddTransition(SearchStatePtr, PatrolStatePtr, [this, GuardController]()
	{
		UBlackboardComponent* BlackboardComp = GuardController->GetBlackboardComponent();
		if (BlackboardComp == nullptr || Guard == nullptr)
		{
			return false;
		}

		const float SearchStartTime = BlackboardComp->GetValueAsFloat(SearchStartTimeKey);
		return Guard->GetWorld()->GetTimeSeconds() - SearchStartTime >= SearchDuration;
	});

	GuardController->RunFiniteStateMachine();
}

void ALevel_FSM::BindLevelInputActions()
{
	Super::BindLevelInputActions();

	if (PlayerEnhancedInputComponent != nullptr && SetThiefTargetAction != nullptr)
	{
		PlayerEnhancedInputComponent->BindAction(
			SetThiefTargetAction,
			ETriggerEvent::Started,
			this,
			&ALevel_FSM::SetThiefTarget);
	}
}

// Called every frame
void ALevel_FSM::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (PlayerController != nullptr && PlayerController->WasInputKeyJustPressed(EKeys::LeftMouseButton))
	{
		SetThiefTarget();
	}

	DrawPatrolRoute();
}

void ALevel_FSM::SetThiefTarget()
{
	if (Thief == nullptr)
	{
		return;
	}

	ThiefMoveBehavior.SetTarget(FTargetData{FVector2D{LatestMouseWorldPos.X, LatestMouseWorldPos.Y}});
	Thief->SetSteeringBehavior(&ThiefMoveBehavior);
}

void ALevel_FSM::DrawPatrolRoute() const
{
	if (PatrolRoute.Num() < 2 || GetWorld() == nullptr)
	{
		return;
	}

	for (int32 Index = 0; Index < PatrolRoute.Num(); ++Index)
	{
		const FVector Start = PatrolRoute[Index];
		const FVector End = PatrolRoute[(Index + 1) % PatrolRoute.Num()];

		DrawDebugSphere(GetWorld(), Start, 20.f, 12, FColor::Green, false, -1.f, 0, 2.f);
		DrawDebugLine(GetWorld(), Start, End, FColor::Green, false, -1.f, 0, 2.f);
	}
}