// Copyright 2019 - 2020 Esri. All Rights Reserved.

#include "VitruvioActor.h"

#include "VitruvioModule.h"

#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "ObjectEditorUtils.h"

namespace
{
int32 CalculateRandomSeed(const FTransform Transform, UStaticMesh* const InitialShape)
{
	FVector Centroid = FVector::ZeroVector;
	if (InitialShape->RenderData != nullptr && InitialShape->RenderData->LODResources.IsValidIndex(0))
	{
		const FStaticMeshLODResources& LOD = InitialShape->RenderData->LODResources[0];
		for (auto SectionIndex = 0; SectionIndex < LOD.Sections.Num(); ++SectionIndex)
		{
			for (uint32 VertexIndex = 0; VertexIndex < LOD.VertexBuffers.PositionVertexBuffer.GetNumVertices(); ++VertexIndex)
			{
				Centroid += LOD.VertexBuffers.PositionVertexBuffer.VertexPosition(VertexIndex);
			}
		}
		Centroid = Centroid / LOD.GetNumVertices();
	}
	return GetTypeHash(Transform.TransformPosition(Centroid));
}
} // namespace

AVitruvioActor::AVitruvioActor()
{
	PrimaryActorTick.bCanEverTick = true;

	static ConstructorHelpers::FObjectFinder<UMaterial> Opaque(TEXT("Material'/Vitruvio/Materials/M_OpaqueParent.M_OpaqueParent'"));
	static ConstructorHelpers::FObjectFinder<UMaterial> Masked(TEXT("Material'/Vitruvio/Materials/M_MaskedParent.M_MaskedParent'"));
	static ConstructorHelpers::FObjectFinder<UMaterial> Translucent(TEXT("Material'/Vitruvio/Materials/M_TranslucentParent.M_TranslucentParent'"));
	OpaqueParent = Opaque.Object;
	MaskedParent = Masked.Object;
	TranslucentParent = Translucent.Object;
}

void AVitruvioActor::BeginPlay()
{
	Super::BeginPlay();
}

void AVitruvioActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// Note that we also tick in editor for initialization
	if (!Initialized)
	{
		// Load default values for generate attributes if they have not been set
		UStaticMesh* InitialShape = GetStaticMeshComponent()->GetStaticMesh();
		if (Rpk && InitialShape)
		{
			LoadDefaultAttributes(InitialShape);
		}

		Initialized = true;
	}
}

void AVitruvioActor::Generate()
{
	if (!Rpk || !AttributesReady || !GetStaticMeshComponent())
	{
		return;
	}

	// Since we can not abort an ongoing generate call from PRT, we just regenerate after the current generate call has completed.
	if (bIsGenerating)
	{
		bNeedsRegenerate = true;
		return;
	}

	bIsGenerating = true;

	UStaticMesh* InitialShape = GetStaticMeshComponent()->GetStaticMesh();

	if (InitialShape)
	{
		TFuture<FGenerateResult> GenerateFuture =
			VitruvioModule::Get().GenerateAsync(InitialShape, OpaqueParent, MaskedParent, TranslucentParent, Rpk, Attributes, RandomSeed);

		// clang-format off
		GenerateFuture.Next([=](const FGenerateResult& Result)
		{
			const FGraphEventRef CreateMeshTask = FFunctionGraphTask::CreateAndDispatchWhenReady([this, &Result]()
			{
				bIsGenerating = false;
				
				// If we need a regenerate (eg there has been a generate request while there 
				if (bNeedsRegenerate)
				{
					bNeedsRegenerate = false;
					Generate();
				}
				else
				{
					// Remove previously generated actors
					TArray<AActor*> GeneratedMeshes;
					GetAttachedActors(GeneratedMeshes);
					for (const auto& Child : GeneratedMeshes)
					{
						Child->Destroy();
					}

					// Create actors for generated meshes
					QUICK_SCOPE_CYCLE_COUNTER(STAT_VitruvioActor_CreateActors);
					FActorSpawnParameters Parameters;
					Parameters.Owner = this;
					AStaticMeshActor* StaticMeshActor = GetWorld()->SpawnActor<AStaticMeshActor>(Parameters);
					StaticMeshActor->SetMobility(EComponentMobility::Movable);
					StaticMeshActor->GetStaticMeshComponent()->SetStaticMesh(Result.ShapeMesh);
					StaticMeshActor->AttachToActor(this, FAttachmentTransformRules::KeepRelativeTransform);

					for (const TTuple<Vitruvio::FInstanceCacheKey, TArray<FTransform>> & MeshAndInstance : Result.Instances)
					{
						auto InstancedComponent = NewObject<UHierarchicalInstancedStaticMeshComponent>(StaticMeshActor);
						const TArray<FTransform>& Instances = MeshAndInstance.Value;
						const Vitruvio::FInstanceCacheKey& CacheKey = MeshAndInstance.Key;
						InstancedComponent->SetStaticMesh(CacheKey.Mesh);

						// Add all instance transforms
						for (const FTransform& Transform : Instances)
						{
							InstancedComponent->AddInstance(Transform);
						}

						// Apply override materials
						for (int32 MaterialIndex = 0; MaterialIndex < CacheKey.MaterialOverrides.Num(); ++MaterialIndex)
						{
							InstancedComponent->SetMaterial(MaterialIndex, CacheKey.MaterialOverrides[MaterialIndex]);
						}
						
						InstancedComponent->AttachToComponent(StaticMeshActor->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
						StaticMeshActor->AddInstanceComponent(InstancedComponent);
					}
					StaticMeshActor->RegisterAllComponents();

					if (HideAfterGeneration)
					{
						GetStaticMeshComponent()->SetVisibility(false);
						SetActorHiddenInGame(true);
					}
					
					bNeedsRegenerate = false;
				}
				
			},
			TStatId(), nullptr, ENamedThreads::GameThread);

			FTaskGraphInterface::Get().WaitUntilTaskCompletes(CreateMeshTask);
		});
		// clang-format on
	}
}

#if WITH_EDITOR

void AVitruvioActor::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	bool bGenerate = GenerateAutomatically; // allow control over generate() in case we trigger it in LoadDefaultAttributes()

	if (PropertyChangedEvent.Property && PropertyChangedEvent.Property->GetFName() == GET_MEMBER_NAME_CHECKED(AVitruvioActor, Rpk))
	{
		Attributes.Empty();

		UStaticMesh* InitialShape = GetStaticMeshComponent()->GetStaticMesh();
		if (Rpk && InitialShape)
		{
			LoadDefaultAttributes(InitialShape);
			bGenerate = false;
		}
	}

	if (PropertyChangedEvent.Property && PropertyChangedEvent.Property->GetFName() == GET_MEMBER_NAME_CHECKED(AVitruvioActor, RandomSeed))
	{
		bValidRandomSeed = true;
	}

	if (PropertyChangedEvent.Property && PropertyChangedEvent.Property->GetFName() == TEXT("StaticMeshComponent"))
	{
		UStaticMesh* InitialShape = GetStaticMeshComponent()->GetStaticMesh();
		if (InitialShape)
		{
			InitialShape->bAllowCPUAccess = true;
			if (Rpk)
			{
				LoadDefaultAttributes(InitialShape);
				bGenerate = false;
			}

			if (!bValidRandomSeed)
			{
				RandomSeed = CalculateRandomSeed(GetActorTransform(), InitialShape);
				bValidRandomSeed = true;
			}
		}
	}

	if (bGenerate)
	{
		Generate();
	}
}

bool AVitruvioActor::ShouldTickIfViewportsOnly() const
{
	return true;
}

#endif // WITH_EDITOR

void AVitruvioActor::LoadDefaultAttributes(UStaticMesh* InitialShape)
{
	check(InitialShape);
	check(Rpk);

	AttributesReady = false;

	TFuture<FAttributeMap> AttributesFuture = VitruvioModule::Get().LoadDefaultRuleAttributesAsync(InitialShape, Rpk, RandomSeed);
	AttributesFuture.Next([this](const FAttributeMap& Result) {
		Attributes = Result.Attributes;
		AttributesReady = true;

		FPlatformMisc::MemoryBarrier();

#if WITH_EDITOR
		// Notify possible listeners (eg. Details panel) about changes to the Attributes
		FFunctionGraphTask::CreateAndDispatchWhenReady(
			[this, &Result]() {
				FPropertyChangedEvent PropertyEvent(GetClass()->FindPropertyByName(GET_MEMBER_NAME_CHECKED(AVitruvioActor, Attributes)));
				FCoreUObjectDelegates::OnObjectPropertyChanged.Broadcast(this, PropertyEvent);
			},
			TStatId(), nullptr, ENamedThreads::GameThread);
#endif // WITH_EDITOR

		if (GenerateAutomatically)
		{
			Generate();
		}
	});
}