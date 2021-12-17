/* Copyright 2021 Esri
 *
 * Licensed under the Apache License Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include "PolygonWindings.h"

#include "Engine/Polys.h"

namespace
{
struct FWindingEdge
{
	const int32 Index0;
	const int32 Index1;

	int32 Count;

	FWindingEdge() : Index0(-1), Index1(-1), Count(0) {}

	FWindingEdge(const int32 Index0, const int32 Index1) : Index0(Index0), Index1(Index1), Count(1) {}

	bool operator==(const FWindingEdge& E) const
	{
		return (E.Index0 == Index0 && E.Index1 == Index1) || (E.Index0 == Index1 && E.Index1 == Index0);
	}
};

uint32 GetTypeHash(const FWindingEdge& Edge)
{
	// Use a modified version of cantor paring to make the hash function commutative. See
	// https://math.stackexchange.com/questions/882877/produce-unique-number-given-two-integers
	const int32 MaxIndex = FMath::Max(Edge.Index0, Edge.Index1);
	const int32 MinIndex = FMath::Min(Edge.Index0, Edge.Index1);
	return (MaxIndex * (MaxIndex + 1)) / 2 + MinIndex;
}

bool IsInsidePoly(const TArray<FVector>& Vertices, const FVector& Vertex)
{
	static const FVector TRACE_OFFSET = {1e6, 0, 0};
	const FVector TraceEndPoint = Vertex + TRACE_OFFSET;

	if (Vertices.Num() < 3)
	{
		return false;
	}

	int NumIntersections = 0;
	for (int32 VertexIndex = 0; VertexIndex < Vertices.Num(); VertexIndex++)
	{
		const FVector SideEnd =  Vertices[(VertexIndex + 1 >= Vertices.Num()) ? 0 : VertexIndex + 1];
		FVector TraceResult;
		const bool Intersection = FMath::SegmentIntersection2D(Vertices[VertexIndex], SideEnd, Vertex, TraceEndPoint, TraceResult);
		if (Intersection)
		{
			NumIntersections++;
		}
	}

	return NumIntersections % 2 == 1;
}

bool IsInsideOf(const TArray<FVector>& FaceA, const TArray<FVector>& FaceB)
{
	if (FaceB.Num() == 1)
	{
		return false;
	}
	
	for (int IndexA = 0; IndexA < FaceA.Num() - 1; IndexA++)
	{
		for (int IndexB = 0; IndexB < FaceB.Num() - 1; IndexB++)
		{
			FVector Intersection;
			const bool Intersected = FMath::SegmentIntersection2D(FaceA[IndexA], FaceA[IndexA + 1],
				FaceB[IndexB], FaceB[IndexB + 1], Intersection);
			if (Intersected)
			{
				return false;
			}
		}
	}

	return IsInsidePoly(FaceB, FaceA[0]);
}

} // namespace

namespace Vitruvio
{

FPolygon GetPolygon(const TArray<FVector>& InVertices, const TArray<int32>& InIndices)
{
	const int32 NumTriangles = InIndices.Num() / 3;

	// Create a list of edges and count the number of times they are used
	TSet<FWindingEdge> Edges;
	for (int32 TriangleIndex = 0; TriangleIndex < NumTriangles; ++TriangleIndex)
	{
		for (int32 VertexIndex = 0; VertexIndex < 3; ++VertexIndex)
		{
			const int32 Index0 = InIndices[TriangleIndex * 3 + VertexIndex];
			const int32 Index1 = InIndices[TriangleIndex * 3 + (VertexIndex + 1) % 3];

			const FWindingEdge Edge(Index0, Index1);
			FWindingEdge* ExistingEdge = Edges.Find(Edge);
			if (ExistingEdge)
			{
				ExistingEdge->Count++;
			}
			else
			{
				Edges.Add(Edge);
			}
		}
	}

	// We only need edges which are used exactly once, this will leave edges at the outside of the polygon or a hole
	TSortedMap<int32, FWindingEdge> EdgeMap;
	for (const FWindingEdge& Edge : Edges)
	{
		if (Edge.Count == 1)
		{
			EdgeMap.Add(Edge.Index0, Edge);
		}
	}

	// Organize the remaining edges in the list so that the vertices will meet up to form a continuous outline around the polygon or hole
	TArray<TArray<FVector>> Windings;
	while (EdgeMap.Num() > 0)
	{
		TArray<FVector> WindingVertices;

		// Get and remove first edge
		auto EdgeIter = EdgeMap.CreateIterator();
		const FWindingEdge FirstEdge = EdgeIter.Value();
		EdgeIter.RemoveCurrent();

		WindingVertices.Add(InVertices[FirstEdge.Index0]);
		int NextIndex = FirstEdge.Index1;

		// Find connected edges
		while (EdgeMap.Contains(NextIndex))
		{
			const FWindingEdge& Current = EdgeMap.FindAndRemoveChecked(NextIndex);
			WindingVertices.Add(InVertices[Current.Index0]);
			NextIndex = Current.Index1;
		}

		Windings.Add(WindingVertices);
	}

	// Find the relation between the faces
	TMap<int32, int32> InsideOf;
	for (int32 IndexA = 0; IndexA < Windings.Num(); IndexA++)
	{
		for (int32 IndexB = IndexA + 1; IndexB < Windings.Num(); IndexB++)
		{
			if (IsInsideOf(Windings[IndexA], Windings[IndexB]))
			{
				InsideOf.Add(IndexA, IndexB);
			}
			
			if (IsInsideOf(Windings[IndexB], Windings[IndexA]))
			{
				InsideOf.Add(IndexB, IndexA);
			}
		}
	}

	TSet<int32> OpenSet;
	for (int32 FaceIndex = 0; FaceIndex < Windings.Num(); ++FaceIndex)
	{
		OpenSet.Add(FaceIndex);
	}
	
	TMap<int32, FFace> FaceMap;
	
	// First find the outermost faces
	for (int32 FaceIndex = 0; FaceIndex < Windings.Num(); ++FaceIndex)
	{
		if (!InsideOf.Contains(FaceIndex))
		{
			FaceMap.Add(FaceIndex, FFace { Windings[FaceIndex] });
			OpenSet.Remove(FaceIndex);
		}
	}

	// Then Continuously search for nested faces or holes
	while (!OpenSet.IsEmpty())
	{
		TSet<int32> NewOpen = OpenSet;
		for (int32 FaceIndex : OpenSet)
		{
			if (!InsideOf.Contains(FaceIndex))
			{
				continue;
			}

			int32 InsideOfIndex = InsideOf[FaceIndex];
			if (FaceMap.Contains(InsideOfIndex))
			{
				FFace& ParentFace = FaceMap[InsideOfIndex];
				ParentFace.Holes.Add({Windings[FaceIndex]});
				NewOpen.Remove(FaceIndex);
			}
			else
			{
				// Not a hole because it is not inside a face
				FaceMap.Add(FaceIndex, FFace { Windings[FaceIndex] });
				NewOpen.Remove(FaceIndex);
			}
		}
		OpenSet = NewOpen;
	}

	FPolygon Result;
	for (const auto& Face : FaceMap)
	{
		Result.Faces.Add(Face.Value);
	}
	return Result;
}
} // namespace Vitruvio
