// Copyright 2019 - 2020 Esri. All Rights Reserved.

#pragma once

#include "RuleAttributes.h"
#include "RulePackage.h"
#include "VitruvioModule.h"

#include "CoreMinimal.h"
#include "Engine/StaticMeshActor.h"
#include "Materials/Material.h"
#include "VitruvioComponent.h"
#include "VitruvioActor.generated.h"

UCLASS()
class VITRUVIO_API AVitruvioActor : public AStaticMeshActor
{
	GENERATED_BODY()

public:
	AVitruvioActor();

	UPROPERTY(VisibleAnywhere)
	UVitruvioComponent* VitruvioComponent;
};
