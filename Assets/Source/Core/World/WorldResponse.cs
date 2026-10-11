using System;

namespace Tmnf.Sim
{
    /// <summary>The tick: detection, response, substeps, respawn, export.</summary>
    public sealed partial class World
    {
        // ------------------------------------------------------------------
        // response ctx (the resolver wiring)
        // ------------------------------------------------------------------

        private bool HasStaticBody(uint corpusRef)
        {
            return corpusRef < track.staticResponseBodies.Count
                && track.staticResponsePresent[(int)corpusRef] != 0;
        }

        private CHmsResponseBody? StaticBody(uint corpusRef)
        {
            if (HasStaticBody(corpusRef))
            {
                return track.staticResponseBodies[(int)corpusRef];
            }
            return null;
        }

        /// <summary>The static-body iso resolver used by the vehicle layers.</summary>
        public GmIso4 ResolveBodyIso(uint corpusRef)
        {
            if (corpusRef == playerCorpusRef)
            {
                return dyna.Live().RotPosIso();
            }
            CHmsResponseBody? body = StaticBody(corpusRef);
            if (body.HasValue)
            {
                return body.Value.iso;
            }
            throw new InvalidOperationException("tmnf world: contact delivered to an unknown response body");
        }

        // ------------------------------------------------------------------
        // detection
        // ------------------------------------------------------------------

        /// <summary>compute_surface_pair: one node surface vs one static entry.</summary>
        private bool ComputeSurfacePair(
            int node,
            ref GmIso4 iso1,
            CPlugSurface entrySurface,
            ref GmIso4 entryIso,
            TmnfMeshQueryAccel accel2,
            uint corpus2,
            uint tree2Ref)
        {
            CPlugSurface surface1 = nodes[node].surface;
            if (surface1 == null)
            {
                return false;
            }
            if (entrySurface == null)
            {
                return false;
            }

            // Mergeable buffer when the vehicle shape is a sphere/ellipsoid.
            bool mergeable = surface1.geom is GmSurfGeomSphere || surface1.geom is GmSurfGeomEllipsoid;

            bool bufferIsMergeable = mergeable;
            uint nodeCountBefore = bufferIsMergeable
                ? contactBuffers[node].@base.GetCount()
                : collisionBuffer.GetCount();
            int ok;
            if (mergeable)
            {
                ok = Narrowphase.ComputeSurfaceCollision(
                    surface1, ref iso1, entrySurface, ref entryIso, accel2,
                    contactBuffers[node].@base,
                    Narrowphase.CollisionShapeDispatch.Default());
            }
            else
            {
                ok = Narrowphase.ComputeSurfaceCollision(
                    surface1, ref iso1, entrySurface, ref entryIso, accel2,
                    collisionBuffer,
                    Narrowphase.CollisionShapeDispatch.Default());
            }
            if (ok == 0)
            {
                return false;
            }

            if (bufferIsMergeable && contactBuffers[node].active == 0)
            {
                contactBuffers[node].active = 1;
                mergeBuffers.Add(node);
            }

            uint end = bufferIsMergeable
                ? contactBuffers[node].@base.GetCount()
                : collisionBuffer.GetCount();
            uint corpus1 = currentCorpus1;
            uint tree1Ref = nodes[node].objectRef;
            uint currentMaterialLocal = currentMaterial;
            for (uint i = nodeCountBefore; i < end; i++)
            {
                if (bufferIsMergeable)
                {
                    ref SHmsPhysicalCollision record = ref contactBuffers[node].@base.collisions.At((int)i);
                    record.corpus1 = corpus1;
                    record.tree1 = tree1Ref;
                    record.corpus2 = corpus2;
                    record.tree2 = tree2Ref;
                    record.material = currentMaterialLocal;
                }
                else
                {
                    ref SHmsPhysicalCollision record = ref collisionBuffer.collisions.At((int)i);
                    record.corpus1 = corpus1;
                    record.tree1 = tree1Ref;
                    record.corpus2 = corpus2;
                    record.tree2 = tree2Ref;
                    record.material = currentMaterialLocal;
                }
            }
            return true;
        }

        /// <summary>0x0053A120: the car tree vs the static group.</summary>
        private void StaticTreeDetect(ref GmIso4 iso, int node)
        {
            uint flags = nodes[node].flags;
            if ((flags & 0x80) == 0)
            {
                return;
            }

            GmIso4 world = VehicleTreeWorldIso(nodes[node], ref iso);
            int[] children = nodes[node].children.ToArray();
            foreach (int child in children)
            {
                GmIso4 worldCopy = world;
                StaticTreeDetect(ref worldCopy, child);
            }
            if (nodes[node].surface == null)
            {
                return;
            }

            GmBoxAligned worldBox = default(GmBoxAligned);
            GmBoxAligned nodeBox = nodes[node].boxAligned;
            worldBox.SetMult(ref nodeBox, ref iso);
            if (track.grid.cellCount != 0)
            {
                uint? cell = Narrowphase.StaticGridLocate(track.grid, ref worldBox, nodes[node].surface);
                if (cell.HasValue)
                {
                    StaticGridScan(cell.Value, ref worldBox, node, ref world);
                    return;
                }
            }

            int entryCount = track.entries.Count;
            int i = 0;
            while (i < entryCount)
            {
                HmsStaticCollisionEntry entry = track.entries[i];
                if (!worldBox.TestInter(ref entry.boxAligned))
                {
                    i += (int)entry.skipCount;
                    continue;
                }
                if (entry.surface >= 0 && (entry.treeFlags & 0x80) != 0)
                {
                    CPlugSurface surface = track.surfaces[entry.surface];
                    GmIso4 entryIso = entry.iso;
                    ComputeSurfacePair(
                        node, ref world, surface, ref entryIso, null,
                        entry.corpusRef, entry.treeRef);
                }
                i += 1;
            }
        }

        /// <summary>static_grid_scan: one cell of the region-restricted grid.</summary>
        private void StaticGridScan(uint cell, ref GmBoxAligned worldQuery, int node, ref GmIso4 world)
        {
            var grid = track.grid;
            int nodesOff = (int)grid.cellOffsets[cell];
            int count = (int)grid.cellCounts[cell];
            int i = 0;
            while (i < count)
            {
                TmnfStaticCellNode cellNode = grid.nodes[nodesOff + i];
                if (!worldQuery.TestInter(ref cellNode.boxAligned))
                {
                    i += (int)cellNode.skip;
                    continue;
                }
                i += 1;
                if (cellNode.entryIndex >= CellNodeConstants.TmnfCellNodeEmpty)
                {
                    continue;
                }
                int entryIndex = (int)cellNode.entryIndex;
                HmsStaticCollisionEntry entry = track.entries[entryIndex];
                int? meshGridIndex = grid.entryMeshGrids[entryIndex];
                GmIso4 inverseIso = grid.entryInverseIsos[entryIndex];
                TmnfMeshQueryAccel accel = new TmnfMeshQueryAccel();
                accel.inverseIso = inverseIso;
                accel.meshGrid = meshGridIndex.HasValue ? grid.meshGrids[meshGridIndex.Value] : null;
                CPlugSurface surface = entry.surface >= 0 ? track.surfaces[entry.surface] : null;
                GmIso4 entryIso = entry.iso;
                ComputeSurfacePair(
                    node, ref world, surface, ref entryIso, accel,
                    entry.corpusRef, entry.treeRef);
            }
        }

        /// <summary>0x0053B1C0: DetectCollisionsCorpus for the car corpus.</summary>
        private void DetectCollisionsCorpus(bool active)
        {
            if (active)
            {
                currentMaterial = 1;
                staticGroupBound = true;
                currentCorpus1 = playerCorpusRef;
            }
            GmIso4 corpusIso = dyna.Live().RotPosIso();
            int[] rootChildrenLocal = rootChildren.ToArray();
            foreach (int child in rootChildrenLocal)
            {
                GmIso4 isoCopy = corpusIso;
                StaticTreeDetect(ref isoCopy, child);
            }

            if (!active)
            {
                return;
            }
            var merge = mergeBuffers.ToArray();
            mergeBuffers.Clear();
            foreach (int nodeIndex in merge)
            {
                Narrowphase.ShmsSphereBufferContactMergeAndAddToCollisions(
                    contactBuffers[nodeIndex], collisionBuffer);
            }
        }

        // ------------------------------------------------------------------
        // the tick
        // ------------------------------------------------------------------

        /// <summary>
        /// Diagnostic hook (env-gated, non-behavioral): prints force/torque
        /// per ComputeCorpusForces call when TMNF_DBG is set — local
        /// divergence hunting only, never in the golden path.
        /// </summary>
        private static bool DbgEnabled()
        {
            return DbgHook.Enabled;
        }

        /// <summary>0x005481A0: ComputeCorpusForces.</summary>
        private void ComputeCorpusForces(float dt)
        {
            dyna.ValidateDynamicState();
            if ((sceneFlags & 0x00100000) != 0)
            {
                GmVec3 zero = GmVec3.Zero;
                dyna.SetForce(ref zero);
                dyna.SetTorque(ref zero);
                return;
            }

            GmVec3 force = GmVec3.Zero;
            GmVec3? gravity = Gravity();
            if (gravity.HasValue)
            {
                GmVec3 value = gravity.Value;
                float scale = (float)(double)(dyna.@params.forceFieldScale * dyna.@params.mass);
                force.x = (float)(double)(((float)(double)(scale * value.x)) + force.x);
                force.y = (float)(double)(((float)(double)(value.y * scale)) + force.y);
                force.z = (float)(double)(((float)(double)(scale * value.z)) + force.z);
            }

            GmVec3 linearSpeed = dyna.GetLinearSpeed();
            float linearDrag = -((float)(double)(linearDragScale * dyna.@params.dragLinear));
            force.x = (float)(double)(((float)(double)(linearDrag * linearSpeed.x)) + force.x);
            force.y = (float)(double)(((float)(double)(linearSpeed.y * linearDrag)) + force.y);
            force.z = (float)(double)(((float)(double)(linearDrag * linearSpeed.z)) + force.z);
            dyna.SetForce(ref force);

            if (dyna.mode == 1)
            {
                GmVec3 angularSpeed = dyna.GetAngularSpeed();
                float angularDrag = -((float)(double)(angularDragScale * dyna.@params.dragAngular));
                GmVec3 torque = new GmVec3(
                    (float)(double)(angularDrag * angularSpeed.x),
                    (float)(double)(angularSpeed.y * angularDrag),
                    (float)(double)(angularDrag * angularSpeed.z));
                dyna.SetTorque(ref torque);
            }

            // The vehicle force pipeline (TMNFVehicleComputeForces adapter).
            {
                Model6External ext = new Model6External
                {
                    modelIsoSource = dyna.Live().RotPosIso(),
                    bodyReferencePosition = cold.bodyReferencePosition,
                };
                TmnfTrackWater water = track.water;
                VehicleCompute.ComputeExternal external = new VehicleCompute.ComputeExternal
                {
                    ext = ext,
                    water = water,
                    postForce = ctx0 => PostVehicleForce(ctx0),
                    contactToken = corpus => corpus,
                    bodyIso = ResolveBodyIso,
                    fakeContactsActive = false,
                    waterForcesActive = false,
                };
                VehicleCtx ctx = new VehicleCtx(vehicle, cold.tuning, dyna);
                VehicleCompute.CsceneVehicleCarComputeForces(ref ctx, external, dt);
            }

            // set_surface_location for every wheel whose suspension
            // integrated inside the force pass.
            SyncWheelSurfaces();
            if (DbgEnabled())
            {
                GmVec3 f = dyna.Live().force;
                GmVec3 t = dyna.Live().torque;
                Console.Error.WriteLine(
                    $"CCF tick={tickTime} dt={BitConverter.SingleToInt32Bits(dt):X8} " +
                    $"force=({BitConverter.SingleToInt32Bits(f.x):X8},{BitConverter.SingleToInt32Bits(f.y):X8},{BitConverter.SingleToInt32Bits(f.z):X8}) " +
                    $"torque=({BitConverter.SingleToInt32Bits(t.x):X8},{BitConverter.SingleToInt32Bits(t.y):X8},{BitConverter.SingleToInt32Bits(t.z):X8})");
            }
        }

        private void PostVehicleForce(VehicleCtx ctx)
        {
            // post_vehicle_force.
            ctx.vehicle.engine.brakingFactor = ctx.vehicle.compute.computedBrakeForce;
        }

        /// <summary>step_dynamic_corpus: one adaptive integration/collision sequence.</summary>
        private void StepDynamicCorpus(float dt)
        {
            bool active = dyna.dirtyFlag != 0;
            uint substeps = 0;
            float remaining = dt;
            float step = 0.0f;
            if (active)
            {
                dyna.CopyStateToTemp();

                GmVec3 linearSpeed = dyna.GetLinearSpeed();
                GmVec3 angularSpeed = dyna.GetAngularSpeed();
                float speedSum = (float)(double)(Length3(ref linearSpeed) + Length3(ref angularSpeed));
                float numerator = (float)(double)(dt * speedSum);
                float quotient = (float)(double)(numerator / dyna.@params.substepLen);
                substeps = unchecked((uint)Fp.Ftol((double)quotient) + 1);
                if (substeps > 1000)
                {
                    substeps = 1000;
                }
                if (substeps > 1)
                {
                    step = (float)((double)dt / (double)substeps);
                }
            }
            if (DbgEnabled())
            {
                Console.Error.WriteLine(
                    $"SDC tick={tickTime} substeps={substeps} " +
                    $"step={BitConverter.SingleToInt32Bits(step):X8} " +
                    $"remaining={BitConverter.SingleToInt32Bits(remaining):X8}");
            }

            uint rounds = substeps;
            for (uint i = 1; i < rounds; i++)
            {
                bool stepping = active && i < substeps;
                if (stepping)
                {
                    ComputeCorpusForces(step);
                    dyna.DoPreCollisionDynamic(step);
                }
                DetectAndRespond(stepping);
                if (stepping)
                {
                    dyna.DoPostCollisionDynamic();
                    remaining = (float)(double)(remaining - step);
                }
            }

            if (active)
            {
                ComputeCorpusForces(remaining);
                dyna.DoPreCollisionDynamic(remaining);
            }
            DetectAndRespond(active);
            if (active)
            {
                dyna.DoPostCollisionDynamic();
                dyna.CopyTempToState();
            }
        }

        /// <summary>detect_and_respond: one detection + response pass.</summary>
        private void DetectAndRespond(bool active)
        {
            if (active)
            {
                collisionBuffer.collisions.Clear();
            }
            DetectCollisionsCorpus(active);
            if (!active)
            {
                return;
            }
            // The route's waypoint triggers stand in for the game's
            // checkpoint corpora.
            if (route != null)
            {
                GmIso4 corpusIso = dyna.Live().RotPosIso();
                CPlugTree[] carTrees = CarTreeSnapshot();
                ulong mask = Race.TmnfRaceTriggerContactMask(route, carTrees, 0, ref corpusIso);
                triggerContacts |= mask;
            }
            Response.ComputeCollisionResponse(new WorldResponseCtxAdapter(this));
        }

        /// <summary>0x00549C90: PhysicsStep2 — one physics tick.</summary>
        public void PhysicsStep2(uint tickMs)
        {
            float dt = (float)(double)(unchecked((int)tickMs) * 0.001f);
            triggerContacts = 0;

            // Initial force/integration pass over the zone's dynamic corpora.
            {
                uint sceneFlagsLocal = sceneFlags;
                if ((sceneFlagsLocal & 0x0001E000) == 0)
                {
                    ComputeCorpusForces(dt);
                    dyna.DoPreCollisionDynamic(dt);
                }
            }

            // Group 0: the car corpus.
            StepDynamicCorpus(dt);
        }

        private static float Length3(ref GmVec3 v)
        {
            return Fp.Sqrt((float)(double)(((float)(double)(((float)(double)(v.y * v.y)) + ((float)(double)(v.x * v.x)))) + ((float)(double)(v.z * v.z))));
        }

        // ------------------------------------------------------------------
        // respawn (vehicle_respawn.c)
        // ------------------------------------------------------------------

        private static void WheelStateReset(ref CSceneVehicleCarWheelState state)
        {
            // 0x007BCA20: zero the listed dwords, the 16-bit word at +0x0c,
            // and store the identity matrix at +0x30.
            uint[] zeroed =
            {
                0x00, 0x04, 0x08, 0x10, 0x14, 0x18, 0x1c, 0x20, 0x24, 0x28,
                0x2c, 0x54, 0x58, 0x5c, 0x60,
            };
            foreach (uint o in zeroed)
            {
                BitsUtil.PutU32(state.bytes, (int)o, 0);
            }
            BitsUtil.PutU16(state.bytes, 0x0c, 0);
            float one = 1.0f;
            float zero = 0.0f;
            for (int i = 0; i < 9; i++)
            {
                float src = i % 4 == 0 ? one : zero;
                BitsUtil.PutF32(state.bytes, 0x30 + 4 * i, src);
            }
        }

        /// <summary>0x0047BF00: reset the car and place it at the spawn.</summary>
        public void Respawn(ref GmIso4 spawn)
        {
            // vehicle_reset (0x007C0320 + base 0x007CB6B0).
            {
                float groundedForceFieldScale = cold.tuning.compute.groundedForceFieldScale;

                Vehicle v = vehicle;
                v.inputGas = 0.0f;
                v.inputBrake = 0.0f;
                v.inputSteer = 0.0f;
                v.compute.airEffectThreshold = 0.0f;
                v.compute.springA.value = 0.0f;
                v.compute.springA.target = 0.0f;
                v.compute.springA.velocity = 0.0f;
                v.compute.springC.value = 0.0f;
                v.compute.springC.target = 0.0f;
                v.compute.springC.velocity = 0.0f;
                v.compute.contactRise = 0.0f;
                v.compute.contactDecay = 0.0f;
                _ = groundedForceFieldScale;
            }
            DynaReset(dyna);
            {
                Vehicle v = vehicle;
                v.steeringValue = 0.0f;
                v.turboProgress = 0.0f;
                v.turboType = TMNFVehicleTurboType.None;
                v.compute.airImpulseCooldownTick = 0;
                v.airControlImmediate = 0;
                v.compute.state5d8 = 0;
                v.model6.contactBlockCount = 0;
                v.sideContact = 0;
                v.model6.sideContact = 0;
                v.airControlTick = 0;
                v.airControlSpeed = GmVec3.Zero;
                v.turboFactor = 0.0f;
                v.turboActive = 0;
                v.lastSideContactTick = uint.MaxValue;
                v.model6.lastSlidingTick = uint.MaxValue;
                v.model6.slidingStartTick = uint.MaxValue;
                v.airControlLocked = 0;
                v.airborneFrictionGate = 0;
                v.forceWheelSpeed = 0;
                v.model6.burnoutStartTick = uint.MaxValue;
                v.model6.burnoutTransitionTick = uint.MaxValue;
                v.model6.modelIso = GmIso4.Identity;
                v.model6.orbitAxis = GmVec3.Zero;
                v.currentLocalSpeed = GmVec3.Zero;
                v.compute.effectAccumulator = 0.0f;
                v.driveMode = 0;
                v.engineMode = 0;
                v.engineLimitFlag = 0;
                v.blockWheelSpeed = 0;
                v.compute.eventLevelA = 0;
                v.compute.eventLevelB = 0;
                v.compute.eventLevelC = 0;
                v.compute.eventSourceAb = 0;
                v.compute.eventSourceC = 0;
                v.compute.peakEventLevelB = 0;
                v.compute.peakEventLevelA = 0;
                v.compute.peakEventLevelC = 0;
                v.compute.eventMetricA = 0.0f;
                v.compute.peakEventSourceAb = 0;
                v.compute.eventMetricB = 0.0f;
                v.compute.peakEventSourceC = 0;
                v.compute.eventMetricC = 0.0f;
                v.compute.lastForceTick = 0;
                v.totalForceAdded = GmVec3.Zero;
                v.totalImpulseAdded = GmVec3.Zero;
            }

            // wheel_reset (0x007BD2A0) per wheel.
            float rest = cold.tuning.@base.suspensionRestLength;
            for (int index = 0; index < 4; index++)
            {
                float minusRest = -rest;
                float minusZero = (float)(double)(minusRest * 0.0f);

                CSceneVehicleCarWheel wheel = vehicle.wheels[index];
                wheel.realTime.field08 = 0.0f;
                wheel.realTime.field04 = 0.0f;
                wheel.realTime.damperAbsorb = rest;
                wheel.surfaceLocation = wheel.surfaceSource;
                GmIso4 location = wheel.surfaceLocation;
                location.t0 = (float)(double)(location.t0 + minusZero);
                location.t1 = (float)(double)(location.t1 + minusRest);
                location.t2 = (float)(double)(location.t2 + minusZero);
                wheel.surfaceLocation = location;
                wheel.realTime.field6c = 0.0f;
                wheel.realTime.rotationPhase = 0.0f;
                wheel.realTime.hasGroundContact = 0;
                wheel.realTime.basis1 = GmMat3.Identity;
                wheel.realTime.field54 = GmVec3.Zero;
                wheel.realTime.field90 = GmVec3.Zero;
                wheel.realTime.reserved60 = new byte[0x0c];
                wheel.realTime.blendValue = 0.0f;
                wheel.realTime.blendTarget = 0.0f;
                // 16-bit store: the upper half of the material word is kept.
                wheel.realTime.contactMaterialId =
                    (int)(((uint)wheel.realTime.contactMaterialId & 0xFFFF0000u) | 0u);
                wheel.realTime.isSliding = 0;
                wheel.realTime.groundContactCount = 0;
                wheel.field15c = 0;
                wheel.contactRelativeLocalDistance = GmVec3.Zero;
                CSceneVehicleCarWheelState field234 = wheel.field234;
                CSceneVehicleCarWheelState asyncState = wheel.asyncState;
                CSceneVehicleCarWheelState previousSync = wheel.previousSync;
                CSceneVehicleCarWheelState sync = wheel.sync;
                WheelStateReset(ref field234);
                WheelStateReset(ref asyncState);
                WheelStateReset(ref previousSync);
                WheelStateReset(ref sync);
                wheel.field234 = field234;
                wheel.asyncState = asyncState;
                wheel.previousSync = previousSync;
                wheel.sync = sync;
                vehicle.wheels[index] = wheel;
            }
            SyncWheelSurfaces();

            // SEngine::Reset (0x007BC9A0).
            {
                Vehicle v = vehicle;
                v.engine.reverse = 0;
                v.engine.rpm = 0.0f;
                v.engine.gear = 1;
                v.engine.targetRpm = 0.0f;
                v.engine.brakingFactor = 0.0f;
                v.engine.shiftTimer = 0.0f;
                v.engine.clutch = 1.0f;
            }
            // The dyna model words the force caller rewrites every tick.
            dyna.@params.forceFieldScale = cold.tuning.compute.groundedForceFieldScale;
            dyna.@params.dragLinear = 0.0f;
            {
                Vehicle v = vehicle;
                v.bodyContactPositionSum = GmVec3.Zero;
                v.bodyContactNormalSum = GmVec3.Zero;
                v.bodyContactCount = 0;
                v.wheelContactAbsorbCount = 0;
                v.flag60c = 0;
            }

            DynaReset(dyna);
            // 0x007BC800 VehicleBlockSpeed2Set(0).
            vehicle.integrationFlags &= ~0x20u;
            DynaSetLocation(dyna, ref spawn);
        }

        private static void DynaStateReset(ref CHmsStateDyna state)
        {
            state.linVel = GmVec3.Zero;
            state.angVel = GmVec3.Zero;
            state.force = GmVec3.Zero;
            state.torque = GmVec3.Zero;
            state.linVelAdded = GmVec3.Zero;
            state.tail0 = 0.0f;
            state.tail1 = 0.0f;
            state.tail2 = 0.0f;
            state.tail3 = 0.0f;
            state.tail4 = 0.0f;
        }

        private static void DynaReset(CHmsDyna d)
        {
            CHmsStateDyna live = d.Live();
            DynaStateReset(ref live);
            d.Live() = live;
            CHmsStateDyna stateB = d.StateB();
            DynaStateReset(ref stateB);
            d.SetStateB(stateB);
            CHmsStateDyna temp = d.tempState;
            DynaStateReset(ref temp);
            d.tempState = temp;
            d.replacementBuf.Clear();
        }

        /// <summary>0x005338F0: SetLocation — state B first, then the live copy.</summary>
        private static void DynaSetLocation(CHmsDyna d, ref GmIso4 spawn)
        {
            CHmsStateDyna b = d.StateB();
            GmMat3 spawnRot = default(GmMat3);
            for (int i = 0; i < 9; i++) spawnRot[i] = spawn[i];
            b.quat.SetFromMat3(ref spawnRot);
            b.rot = spawnRot;
            b.pos = new GmVec3(spawn.t0, spawn.t1, spawn.t2);
            // invInertiaWorld = (rot * Iinv_body), then rot^T * itself.
            GmMat3 iw = default(GmMat3);
            GmMat3 rotCopy = b.rot;
            GmMat3 invInertiaBody = d.@params.invInertiaBody;
            iw.SetMult(ref rotCopy, ref invInertiaBody);
            iw.MultTranspose(ref rotCopy);
            b.invInertiaWorld = iw;
            d.SetStateB(b);
            CHmsStateDyna b2 = d.StateB();
            CHmsStateDyna live = d.Live();
            live.quat = b2.quat;
            GmMat3 spawnRot2 = default(GmMat3);
            for (int i = 0; i < 9; i++) spawnRot2[i] = spawn[i];
            live.rot = spawnRot2;
            live.pos = new GmVec3(spawn.t0, spawn.t1, spawn.t2);
            live.invInertiaWorld = b2.invInertiaWorld;
            d.Live() = live;
        }

        // ------------------------------------------------------------------
        // response ctx adapter
        // ------------------------------------------------------------------

        /// <summary>
        /// Bridges the world into <see cref="Response.ComputeCollisionResponse"/>
        /// (the interface cannot express the world's split borrows directly).
        /// </summary>
        private sealed class WorldResponseCtxAdapter : IResponseCtx
        {
            private readonly World world;
            private CHmsResponseBody body1Cache;
            private CHmsResponseBody body2Cache;
            private CHmsResponseMaterial materialCache;

            public WorldResponseCtxAdapter(World world)
            {
                this.world = world;
            }

            CHmsResponseBody IResponseCtx.Body(uint corpusRef)
            {
                if (corpusRef == world.playerCorpusRef)
                {
                    return world.cold.playerResponseBody;
                }
                CHmsResponseBody? body = world.StaticBody(corpusRef);
                if (body.HasValue)
                {
                    return body.Value;
                }
                throw new InvalidOperationException("tmnf world: contact delivered to an unknown response body");
            }

            GmIso4 IResponseCtx.BodyIso(uint corpusRef)
            {
                return world.ResolveBodyIso(corpusRef);
            }

            GmVec3 IResponseCtx.GetSpeed(uint corpusRef, ref GmVec3 position)
            {
                if (corpusRef == world.playerCorpusRef)
                {
                    return ResponseGetSpeed.Compute(ref world.cold.playerResponseBody, world.dyna, ref position);
                }
                CHmsResponseBody? body = world.StaticBody(corpusRef);
                if (body.HasValue)
                {
                    CHmsResponseBody b = body.Value;
                    return ResponseGetSpeed.Compute(ref b, null, ref position);
                }
                throw new InvalidOperationException("tmnf world: contact delivered to an unknown response body");
            }

            void IResponseCtx.AddReplacement(uint corpusRef, ref GmVec3 d)
            {
                if (corpusRef == world.playerCorpusRef)
                {
                    world.dyna.AddReplacement(ref d);
                }
            }

            void IResponseCtx.SolveBodyImpulse(uint corpusRef, ref GmCollision collision, ref GmVec3 speed, float frictionProduct, float restitution)
            {
                if (corpusRef == world.playerCorpusRef)
                {
                    CHmsResponseBody body = world.cold.playerResponseBody;
                    Response.SolveBodyImpulse(ref body, world.dyna, ref collision, ref speed, frictionProduct, restitution);
                }
            }

            void IResponseCtx.AbsorbContact(uint corpusRef, ref CHmsPhysicalContact contact)
            {
                if (corpusRef != world.playerCorpusRef)
                {
                    throw new InvalidOperationException("tmnf world: contact delivered to the wrong response body");
                }
                VehicleCtx ctx = new VehicleCtx(world.vehicle, world.cold.tuning, world.dyna);
                VehicleContact.AbsorbContact(ref ctx, ref contact, world.ResolveBodyIso);
                world.vehicle.model6.sideContact = world.vehicle.sideContact;
            }

            CHmsResponseMaterial IResponseCtx.ResponseMaterial(uint materialRef)
            {
                if (materialRef == 1)
                {
                    return world.cold.responseMaterial;
                }
                throw new InvalidOperationException("tmnf world: unknown response material");
            }

            CPlugSurfaceMaterialData[] IResponseCtx.SurfaceMaterials()
            {
                return world.track.materials.ToArray();
            }

            CFastBufferShmsPhysicalCollision IResponseCtx.Collisions()
            {
                return world.collisionBuffer.collisions;
            }
        }
    }
}
