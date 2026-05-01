// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Shared/Level_Base.h"
#include "GameAIProg/Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
#include "Level_FSM.generated.h"

UCLASS()
class GAMEAIPROG_API ALevel_FSM : public ALevel_Base
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="FSM|Config")
	float DetectionRadius{500.f};

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="FSM|Config")
	float SearchDuration{4.f};

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="FSM|Config")
	float PatrolPointAcceptanceRadius{75.f};

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="FSM|Config")
	float SearchArrivalRadius{100.f};

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="FSM|Config")
	TArray<FVector> PatrolRoute{};

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="FSM|Sight")
	float LoseSightRadiusMultiplier{1.2f};

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="FSM|Sight")
	float PeripheralVisionAngleDegrees{60.f};

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="FSM|Thief")
	float ThiefRetargetInterval{1.25f};

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="FSM|Thief")
	float ThiefWanderRadius{500.f};

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="FSM|Thief")
	float ThiefKeepAwayDistance{350.f};

	ALevel_FSM();

	virtual void Tick(float DeltaTime) override;

protected:
	virtual void BeginPlay() override;

private:
	UPROPERTY()
	ASteeringAgent* Guard{nullptr};

	UPROPERTY()
	ASteeringAgent* Thief{nullptr};

	Arrive ThiefMoveBehavior{};
	float NextThiefRetargetTime{0.f};

	void UpdateThiefAI();
	FVector GetNextThiefTargetLocation() const;
	void DrawPatrolRoute() const;
};