// Fill out your copyright notice in the Description page of Project Settings.

#include "PRTActor.h"
#include "UnrealGeometryEncoderModule.h"

APRTActor::APRTActor()
{
	PrimaryActorTick.bCanEverTick = true;
}

void APRTActor::BeginPlay()
{
	Super::BeginPlay();
}

void APRTActor::Tick(float DeltaTime)
{
	// Note that we also tick in editor for initialization
	if (!Initialized)
	{
		// Load default values for generate attributes if they have not been set
		if (Rpk)
		{
			UStaticMesh* InitialShape = GetStaticMeshComponent()->GetStaticMesh();
			UnrealGeometryEncoderModule::Get().SetDefaultRuleAttributes(InitialShape, Rpk, GenerateAttributes);
		}

		if (GenerateAutomatically)
		{
			Regenerate();
		}

		Initialized = true;
	}
}

void APRTActor::Regenerate()
{
	if (Rpk)
	{
		UStaticMeshComponent* StaticMeshComponent = GetStaticMeshComponent();

		// Remove old prt meshes
		TArray<AActor*> Children;
		GetAttachedActors(Children);
		for (const auto& Child : Children)
		{
			Child->Destroy();
		}

		// Generate
		if (StaticMeshComponent)
		{
			UStaticMesh* InitialShape = StaticMeshComponent->GetStaticMesh();

			if (InitialShape)
			{
				TArray<UStaticMesh*> Generated = UnrealGeometryEncoderModule::Get().Generate(InitialShape, Rpk, GenerateAttributes);

				for (UStaticMesh* Mesh : Generated)
				{
					FActorSpawnParameters Parameters;
					Parameters.Owner = this;
					AStaticMeshActor* StaticMeshActor = GetWorld()->SpawnActor<AStaticMeshActor>(Parameters);
					StaticMeshActor->SetMobility(EComponentMobility::Movable);
					StaticMeshActor->GetStaticMeshComponent()->SetStaticMesh(Mesh);
					StaticMeshActor->AttachToActor(this, FAttachmentTransformRules::KeepRelativeTransform);
				}
			}
		}
	}
}

#ifdef WITH_EDITOR
void APRTActor::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	if (PropertyChangedEvent.Property->GetFName() == GET_MEMBER_NAME_CHECKED(APRTActor, Rpk))
	{
		GenerateAttributes.Empty();
	}

	if (GenerateAutomatically)
	{
		Regenerate();
	}
}

bool APRTActor::ShouldTickIfViewportsOnly() const
{
	return true;
}
#endif
