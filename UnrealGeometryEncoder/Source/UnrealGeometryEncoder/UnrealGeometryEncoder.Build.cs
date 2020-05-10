// Copyright 2019 - 2020 Esri. All Rights Reserved.

using UnrealBuildTool;

public class UnrealGeometryEncoder : ModuleRules
{
	public UnrealGeometryEncoder(ReadOnlyTargetRules Target) : base(Target)
	{
		bUseRTTI = true;
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
		PrecompileForTargets = PrecompileTargetsType.Any;
		bUseUnity = false;

		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				"CoreUObject",
				"Engine",
				"RHI",
				"RenderCore",
				"PRT",
				"Projects",
				"SlateCore",
				"Slate",
				"AppFramework",
				"CoreUObject",
				"InputCore",
			}
		);

		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"SlateCore",
				"Slate",
				"AppFramework",
			}
		);
	}
}
