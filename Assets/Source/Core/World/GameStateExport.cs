using System;

namespace Tmnf.Sim
{
    /// <summary>
    /// World_WritePlayerGameState: patches every simulated field into the
    /// original image's car/wheel blocks (byte-exact export).
    /// </summary>
    public static class GameStateExport
    {
        /// <summary>Patches the (car, wheels) blocks from the vehicle state.</summary>
        public static void PatchGameState(byte[] car, byte[] wheels, Vehicle vehicle)
        {
            void PutF32(byte[] b, int o, float v) => BitsUtil.PutF32(b, o, v);
            void PutI32(byte[] b, int o, int v) => BitsUtil.PutI32(b, o, v);
            void PutU32(byte[] b, int o, uint v) => BitsUtil.PutU32(b, o, v);
            void PutVec3(byte[] b, int o, ref GmVec3 v)
            {
                PutF32(b, o, v.x);
                PutF32(b, o + 4, v.y);
                PutF32(b, o + 8, v.z);
            }

            // patch_model6_state
            {
                CSceneVehicleCarModel6State s = vehicle.model6;
                PutVec3(car, 0x1dc, ref s.pivotPosition);
                PutVec3(car, 0x1e8, ref s.pivotAxis);
                PutI32(car, 0x5c4, s.reverseMode);
                PutF32(car, 0x5cc, s.reverseSpeedThreshold);
                PutI32(car, 0x5d8, s.contactBlockCount);
                PutI32(car, 0x5dc, s.sideContact);
                PutU32(car, 0x62c, s.lastSlidingTick);
                PutU32(car, 0x630, s.slidingStartTick);
                PutU32(car, 0x634, s.slidingElapsedTicks);
                for (int i = 0; i < 9; i++)
                {
                    PutF32(car, 0x6a4 + 4 * i, s.modelIso[i]);
                }
                for (int i = 0; i < 3; i++)
                {
                    PutF32(car, 0x6a4 + 36 + 4 * i, s.modelIso[9 + i]);
                }
                PutVec3(car, 0x6d4, ref s.rolloverAxis);
                PutVec3(car, 0x6e0, ref s.orbitCenter);
                PutF32(car, 0x6ec, s.orbitInitialRadius);
                PutF32(car, 0x6f0, s.orbitRadius);
                PutU32(car, 0x6f4, s.burnoutStartTick);
                PutU32(car, 0x6f8, s.burnoutTransitionTick);
                PutVec3(car, 0x6fc, ref s.orbitAxis);
                PutF32(car, 0x708, s.orbitSign);
                PutF32(car, 0x840, s.axleWidth);
            }
            // patch_compute_state
            {
                TMNFVehicleComputeState s = vehicle.compute;
                PutF32(car, 0x1e8, s.simulationGate);
                PutU32(car, 0x1fc, s.eventLevelC);
                car[0x200] = s.eventSourceC;
                car[0x201] = s.eventSourceAb;
                PutSpring(car, 0x214, s.springC);
                PutSpring(car, 0x228, s.springA);
                PutF32(car, 0x23c, s.contactRise);
                PutF32(car, 0x240, s.contactDecay);
                PutF32(car, 0x5b0, s.computedBrakeForce);
                PutI32(car, 0x5d8, s.state5d8);
                PutU32(car, 0x610, s.airImpulseCooldownTick);
                PutF32(car, 0x624, s.effectAccumulator);
                PutU32(car, 0x650, s.lastForceTick);
                PutU32(car, 0x654, s.eventLevelA);
                PutU32(car, 0x658, s.eventLevelB);
                PutU32(car, 0x660, s.peakEventLevelB);
                PutU32(car, 0x664, s.peakEventLevelA);
                PutU32(car, 0x668, s.peakEventLevelC);
                car[0x66c] = s.peakEventSourceAb;
                car[0x66d] = s.peakEventSourceC;
                PutF32(car, 0x670, s.eventMetricA);
                PutF32(car, 0x674, s.eventMetricB);
                PutF32(car, 0x678, s.eventMetricC);
                PutVec3(car, 0x6d4, ref s.normalizedForce);
                PutI32(car, 0x74c, s.airEffectMode);
            }

            PutF32(car, 0x050, vehicle.inputGas);
            PutF32(car, 0x054, vehicle.inputBrake);
            PutF32(car, 0x058, vehicle.inputSteer);
            PutF32(car, 0x2e0, vehicle.compute.localSpeedLimit);
            PutI32(car, 0x2e4, vehicle.engineMode);
            PutU32(car, 0x2e8, vehicle.wheelCount);
            PutEngine(car, 0x59c, vehicle.engine);
            PutU32(car, 0x5d0, vehicle.turboEpochTick);
            PutI32(car, 0x5d4, vehicle.airControlImmediate);
            PutI32(car, 0x5d8, vehicle.compute.state5d8);
            PutI32(car, 0x5dc, vehicle.sideContact);
            PutU32(car, 0x5e0, vehicle.lastSideContactTick);
            PutI32(car, 0x5e4, vehicle.airControlLocked);
            PutF32(car, 0x5e8, vehicle.steeringValue);
            PutF32(car, 0x5f0, vehicle.turboProgress);
            PutF32(car, 0x5f4, vehicle.turboFactor);
            PutU32(car, 0x5f8, vehicle.turboStartTick);
            PutU32(car, 0x5fc, vehicle.turboEndTick);
            PutI32(car, 0x600, (int)vehicle.turboType);
            PutU32(car, 0x604, vehicle.rouletteToken);
            PutF32(car, 0x608, vehicle.rouletteValue);
            PutI32(car, 0x60c, vehicle.flag60c);
            PutU32(car, 0x614, vehicle.airControlTick);
            PutVec3(car, 0x618, ref vehicle.airControlSpeed);
            PutI32(car, 0x628, vehicle.turboActive);
            PutI32(car, 0x69c, vehicle.driveMode);
            PutI32(car, 0x6a0, vehicle.forceWheelSpeed);
            PutVec3(car, 0x70c, ref vehicle.currentLocalSpeed);
            PutI32(car, 0x73c, vehicle.blockWheelSpeed);
            PutI32(car, 0x744, vehicle.engineLimitFlag);
            PutI32(car, 0x748, vehicle.gearDownshiftFlag);
            PutVec3(car, 0x818, ref vehicle.totalForceAdded);
            PutVec3(car, 0x824, ref vehicle.totalImpulseAdded);

            for (int i = 0; i < 4; i++)
            {
                CSceneVehicleCarWheel wheel = vehicle.wheels[i];
                int raw = i * World.WheelSize;
                PutI32(wheels, raw + 0x000, wheel.active);
                PutI32(wheels, raw + 0x004, wheel.steerable);
                PutF32(wheels, raw + 0x008, wheel.radius);
                for (int k = 0; k < 9; k++)
                {
                    PutF32(wheels, raw + 0x10 + 4 * k, wheel.surfaceSource[k]);
                }
                PutF32(wheels, raw + 0x10 + 36, wheel.surfaceSource.t0);
                PutF32(wheels, raw + 0x10 + 40, wheel.surfaceSource.t1);
                PutF32(wheels, raw + 0x10 + 44, wheel.surfaceSource.t2);
                for (int k = 0; k < 9; k++)
                {
                    PutF32(wheels, raw + 0x40 + 4 * k, wheel.surfaceLocation[k]);
                }
                PutF32(wheels, raw + 0x40 + 36, wheel.surfaceLocation.t0);
                PutF32(wheels, raw + 0x40 + 40, wheel.surfaceLocation.t1);
                PutF32(wheels, raw + 0x40 + 44, wheel.surfaceLocation.t2);
                for (int k = 0; k < 12; k++)
                {
                    PutF32(wheels, raw + 0x70 + 4 * k, wheel.field70[k]);
                }
                PutI32(wheels, raw + 0xa0, wheel.fielda0);
                PutI32(wheels, raw + 0xa4, wheel.fielda4);
                PutVec3(wheels, raw + 0xa8, ref wheel.offsetFromVehicle);
                PutRealTime(wheels, raw + 0xb4, ref wheel.realTime);
                PutI32(wheels, raw + 0x15c, wheel.field15c);
                PutVec3(wheels, raw + 0x160, ref wheel.contactRelativeLocalDistance);
                Array.Copy(wheel.previousSync.bytes, 0, wheels, raw + 0x16c, 0x64);
                Array.Copy(wheel.sync.bytes, 0, wheels, raw + 0x1d0, 0x64);
                Array.Copy(wheel.field234.bytes, 0, wheels, raw + 0x234, 0x64);
                Array.Copy(wheel.asyncState.bytes, 0, wheels, raw + 0x298, 0x64);
            }
        }

        private static void PutSpring(byte[] b, int o, GmSpringFloat s)
        {
            BitsUtil.PutF32(b, o, s.stiffness);
            BitsUtil.PutF32(b, o + 4, s.damping);
            BitsUtil.PutF32(b, o + 8, s.value);
            BitsUtil.PutF32(b, o + 12, s.target);
            BitsUtil.PutF32(b, o + 16, s.velocity);
        }

        private static void PutEngine(byte[] b, int o, CSceneVehicleCarEngine e)
        {
            // The reference builds the whole 0x30-byte engine block from a
            // ZEROED buffer (engine_bytes) and copies it over the car block,
            // so the reserved words (0x04..0x14) are zeroed by the dump.
            for (int i = 0; i < 0x30; i++)
            {
                b[o + i] = 0;
            }
            BitsUtil.PutF32(b, o + 0x00, e.maxRpm);
            BitsUtil.PutF32(b, o + 0x14, e.brakingFactor);
            BitsUtil.PutF32(b, o + 0x18, e.rpm);
            BitsUtil.PutF32(b, o + 0x1c, e.targetRpm);
            BitsUtil.PutF32(b, o + 0x20, e.clutch);
            BitsUtil.PutF32(b, o + 0x24, e.shiftTimer);
            BitsUtil.PutI32(b, o + 0x28, e.reverse);
            BitsUtil.PutI32(b, o + 0x2c, e.gear);
        }

        private static void PutRealTime(byte[] b, int o, ref CSceneVehicleCarWheelRealTimeState r)
        {
            BitsUtil.PutF32(b, o + 0x00, r.damperAbsorb);
            BitsUtil.PutF32(b, o + 0x04, r.field04);
            BitsUtil.PutF32(b, o + 0x08, r.field08);
            for (int i = 0; i < 9; i++)
            {
                BitsUtil.PutF32(b, o + 0x0c + 4 * i, r.basis0[i]);
            }
            for (int i = 0; i < 9; i++)
            {
                BitsUtil.PutF32(b, o + 0x30 + 4 * i, r.basis1[i]);
            }
            r.field54.ToBytes(b, o + 0x54);
            Array.Copy(r.reserved60, 0, b, o + 0x60, 0x0c);
            BitsUtil.PutF32(b, o + 0x6c, r.field6c);
            BitsUtil.PutI32(b, o + 0x70, r.hasGroundContact);
            BitsUtil.PutI32(b, o + 0x74, r.contactMaterialId);
            BitsUtil.PutI32(b, o + 0x78, r.isSliding);
            r.relativeRotzAxis.ToBytes(b, o + 0x7c);
            BitsUtil.PutU32(b, o + 0x88, r.contactBodyWord);
            BitsUtil.PutI32(b, o + 0x8c, r.groundContactCount);
            r.field90.ToBytes(b, o + 0x90);
            BitsUtil.PutF32(b, o + 0x9c, r.rotationPhase);
            BitsUtil.PutF32(b, o + 0xa0, r.blendValue);
            BitsUtil.PutF32(b, o + 0xa4, r.blendTarget);
        }

        /// <summary>
        /// Dumps the vehicle as the game-state blocks over the image's base
        /// bytes (the golden dump form).
        /// </summary>
        public static (byte[], byte[]) DumpVehicleGameState(byte[] vehicleBlob, Vehicle vehicle)
        {
            // Find the section offsets through the header.
            World.VehicleSnapshotHeader header = World.VehicleSnapshotHeader.FromBytes(vehicleBlob);
            byte[] car = new byte[World.CarSize];
            Array.Copy(vehicleBlob, (int)header.carOffset, car, 0, World.CarSize);
            byte[] wheels = new byte[4 * World.WheelSize];
            Array.Copy(vehicleBlob, (int)header.wheelsOffset, wheels, 0, 4 * World.WheelSize);
            PatchGameState(car, wheels, vehicle);
            return (car, wheels);
        }
    }
}
