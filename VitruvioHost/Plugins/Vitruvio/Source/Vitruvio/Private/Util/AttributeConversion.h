﻿/* Copyright 2021 Esri
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

#include "PRTTypes.h"
#include "RuleAttributes.h"

#include "CoreUObject.h"

namespace Vitruvio
{
TMap<FString, URuleAttribute*> ConvertAttributeMap(const AttributeMapUPtr& AttributeMap, const RuleFileInfoUPtr& RuleInfo, UObject* Outer);
void UpdateAttributeMap(TMap<FString, URuleAttribute*>& AttributeMapOut, const AttributeMapUPtr& AttributeMap, const RuleFileInfoUPtr& RuleInfo, UObject* const Outer);

AttributeMapUPtr CreateAttributeMap(const TMap<FString, URuleAttribute*>& Attributes);
} // namespace Vitruvio
