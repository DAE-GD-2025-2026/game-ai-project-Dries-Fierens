// Fill out your copyright notice in the Description page of Project Settings.

#include "Level_FSM.h"

#include "BehaviorTree/BlackboardComponent.h"
#include "DrawDebugHelpers.h"
#include "FSMComponent.h"
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

	bool IsTargetVisible(AGameAIController* GuardController, ASteeringAgent* Thief)
	{
		return GuardController != nullptr &&
			Thief != nullptr &&
			GuardController->GetTargetActor() == Thief;
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
		FVector{-650.f, 450.f, 90.f},
		FVector{-650.f, -250.f, 90.f},
		FVector{-350.f, -250.f, 90.f},
		FVector{250.f, 150.f, 90.f},
		FVector{250.f, 650.f, 90.f},
		FVector{750.f, 650.f, 90.f},
		FVector{750.f, -650.f, 90.f},
		FVector{-750.f, -650.f, 90.f},
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

	const FVector GuardSpawnLocation = PatrolRoute.IsEmpty() ? FVector{-650.f, 450.f, 90.f} : PatrolRoute[0];
	const FVector ThiefSpawnLocation = FVector{900.f, 700.f, 90.f};

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
	NextThiefRetargetTime = 0.f;

	if (Guard->GetController() == nullptr)
	{
		Guard->SpawnDefaultController();
	}

	AGameAIController* GuardController = Cast<AGameAIController>(Guard->GetController());
	if (!ensure(GuardController != nullptr))
	{
		return;
	}

	GuardController->ConfigureSight(
		DetectionRadius,
		DetectionRadius * LoseSightRadiusMultiplier,
		PeripheralVisionAngleDegrees);

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
			return IsTargetVisible(GuardController, Thief);
		});

	FSM->AddTransition(ChaseStatePtr, SearchStatePtr, [this, GuardController]()
		{
			return !IsTargetVisible(GuardController, Thief);
		});

	FSM->AddTransition(SearchStatePtr, ChaseStatePtr, [this, GuardController]()
		{
			return IsTargetVisible(GuardController, Thief);
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
	UpdateThiefAI();
}

// Called every frame
void ALevel_FSM::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	UpdateThiefAI();
	DrawPatrolRoute();
}

void ALevel_FSM::UpdateThiefAI()
{
	if (Thief == nullptr || GetWorld() == nullptr)
	{
		return;
	}

	const float CurrentTime = GetWorld()->GetTimeSeconds();
	if (CurrentTime < NextThiefRetargetTime)
	{
		return;
	}

	const FVector NextTargetLocation = GetNextThiefTargetLocation();

	ThiefMoveBehavior.SetTarget(FTargetData{FVector2D{NextTargetLocation.X, NextTargetLocation.Y}});
	Thief->SetSteeringBehavior(&ThiefMoveBehavior);

	NextThiefRetargetTime = CurrentTime + ThiefRetargetInterval;
}

FVector ALevel_FSM::GetNextThiefTargetLocation() const
{
	if (Thief == nullptr)
	{
		return FVector::ZeroVector;
	}

	const FVector CurrentLocation = Thief->GetActorLocation();

	FVector Direction{ForceInitToZero};

	if (Guard != nullptr)
	{
		const float DistanceToGuardSq = FVector::DistSquared2D(CurrentLocation, Guard->GetActorLocation());
		if (DistanceToGuardSq <= FMath::Square(ThiefKeepAwayDistance))
		{
			Direction = (CurrentLocation - Guard->GetActorLocation()).GetSafeNormal2D();
		}
	}

	if (Direction.IsNearlyZero())
	{
		const float RandomAngle = FMath::FRandRange(0.f, 2.f * PI);
		Direction = FVector{FMath::Cos(RandomAngle), FMath::Sin(RandomAngle), 0.f};
	}

	const float TravelDistance = FMath::FRandRange(ThiefWanderRadius * 0.5f, ThiefWanderRadius);

	FVector CandidateLocation = CurrentLocation + Direction * TravelDistance;
	CandidateLocation.Z = CurrentLocation.Z;

	if (!PatrolRoute.IsEmpty())
	{
		FBox PatrolBounds{ForceInit};
		for (const FVector& PatrolPoint : PatrolRoute)
		{
			PatrolBounds += PatrolPoint;
		}

		constexpr float BoundsPadding = 250.f;
		CandidateLocation.X = FMath::Clamp(CandidateLocation.X, PatrolBounds.Min.X - BoundsPadding, PatrolBounds.Max.X + BoundsPadding);
		CandidateLocation.Y = FMath::Clamp(CandidateLocation.Y, PatrolBounds.Min.Y - BoundsPadding, PatrolBounds.Max.Y + BoundsPadding);
	}

	return CandidateLocation;
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