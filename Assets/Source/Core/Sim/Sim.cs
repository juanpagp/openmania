using System;

using RaceLayer = Tmnf.Sim.Race;

namespace Tmnf.Sim
{
    /// <summary>Simulation options: where the runtime data comes from.</summary>
    [Serializable]
    public sealed class SimOptions
    {
        /// <summary>Path to the 128x128 fake-contact mask (required).</summary>
        public string fakeContactMask;
        /// <summary>Path to a TMNFROU1 route (null = sandbox).</summary>
        public string route;
        /// <summary>
        /// Route image bytes — the in-memory alternative to
        /// <see cref="route"/>, for callers that build the image themselves
        /// (see Tmnf.Sim.Fixtures.TmnfFixtures). Takes precedence when set.
        /// </summary>
        public byte[] routeBytes;
        /// <summary>Require the vehicle image to pin the canonical hashes.</summary>
        public bool pinProvenance;
    }

    /// <summary>The player input for one tick (the game's 0x48 packet).</summary>
    [Serializable]
    public struct TmnfInput
    {
        public bool accelerate;
        public bool brake;
        public bool steerLeft;
        public bool steerRight;
        /// <summary>Analog gas [-1, 1] (0 = unused).</summary>
        public float gasAnalog;
        /// <summary>Analog steer [-1, 1] (0 = unused).</summary>
        public float steerAnalog;
        /// <summary>Respawn press (needs a route).</summary>
        public bool respawn;

        /// <summary>Digital input: (accelerate, brake, steer_left, steer_right).</summary>
        public static TmnfInput Digital(int accelerate, int brake, int steerLeft, int steerRight)
        {
            TmnfInput input = default(TmnfInput);
            input.accelerate = accelerate != 0;
            input.brake = brake != 0;
            input.steerLeft = steerLeft != 0;
            input.steerRight = steerRight != 0;
            return input;
        }

        /// <summary>Coast (all controls released).</summary>
        public static TmnfInput Idle()
        {
            return default(TmnfInput);
        }

        internal TMNFRaceInputs ToPacket(uint tick)
        {
            // The static 0x004FE500 mapper reads the timestamp/value words;
            // a fresh packet with the current tick as every timestamp
            // reproduces the game's mapping for live input.
            uint t = unchecked(tick * 10);
            TMNFRaceInputs packet = default(TMNFRaceInputs);
            packet.steerLeftTime = steerLeft ? t : 0;
            packet.steerLeft = steerLeft ? 1 : 0;
            packet.steerRightTime = steerRight ? t : 0;
            packet.steerRight = steerRight ? 1 : 0;
            packet.steerAnalogTime = steerAnalog != 0.0f ? t : 0;
            packet.steerAnalog = steerAnalog;
            packet.accelerateTime = accelerate ? t : 0;
            packet.accelerate = accelerate ? 1 : 0;
            packet.brakeTime = brake ? t : 0;
            packet.brake = brake ? 1 : 0;
            packet.gasAnalogTime = gasAnalog != 0.0f ? t : 0;
            packet.respawn = respawn ? 1u : 0;
            packet.gasAnalog = gasAnalog;
            return packet;
        }
    }

    /// <summary>The per-tick observation.</summary>
    [Serializable]
    public struct VehicleState
    {
        /// <summary>Race time in ms since the start (or respawn).</summary>
        public uint raceTimeMs;
        /// <summary>World-space position of the rigid body.</summary>
        public GmVec3 position;
        /// <summary>
        /// Orientation quaternion exposed in (x, y, z, w) field order — the
        /// game's scalar-first (w, x, y, z) byte order mapped to the
        /// conventional accessor order.
        /// </summary>
        public float[] quaternion;
        /// <summary>World-space linear velocity (m/s).</summary>
        public GmVec3 linearSpeed;
        /// <summary>World-space angular velocity (rad/s).</summary>
        public GmVec3 angularSpeed;
        /// <summary>Per-wheel state.</summary>
        public WheelState[] wheels;
        /// <summary>Engine RPM.</summary>
        public float engineRpm;
        /// <summary>Gear (-1 reverse, 1..6 forward).</summary>
        public int gear;
        /// <summary>Whether the next respawn press applies.</summary>
        public bool canRespawn;

        public static VehicleState Create()
        {
            VehicleState s = default(VehicleState);
            s.quaternion = new float[4];
            s.wheels = new WheelState[4];
            return s;
        }
    }

    [Serializable]
    public struct WheelState
    {
        public bool onGround;
        public int contactMaterialId;
        public bool sliding;
        public float spinSpeed;
    }

    /// <summary>One simulation: one car on one track. Step() is one 10 ms tick.</summary>
    public sealed class Sim
    {
        private World world;
        private RouteLoader.TmnfRoute route;
        private RaceLayer.TmnfRaceState race;
        private uint tick;

        /// <summary>Builds a simulation from loaded pieces.</summary>
        public Sim(TrackLoader.TmnfTrack track, byte[] vehicleBlob, byte[] fakeContactMask, SimOptions options)
        {
            if (options.routeBytes != null)
            {
                route = RouteLoader.TmnfRouteLoadBytes(options.routeBytes, null);
            }
            else if (options.route != null)
            {
                route = RouteLoader.TmnfRouteLoad(options.route, null);
            }
            else
            {
                route = null;
            }
            world = new World(track, vehicleBlob, new WorldOptions
            {
                fakeContactMask = fakeContactMask,
                pinProvenance = options.pinProvenance,
            });
            // Rust: race: TmnfRaceState::default() — the derived Default is
            // ALL ZEROS (the respawn/previous-car isos are zero matrices,
            // not identity); reset overwrites them when a route exists.
            race = default(RaceLayer.TmnfRaceState);
            tick = 0;
            if (route != null)
            {
                GmIso4 iso = world.Live().RotPosIso();
                RaceLayer.TmnfRaceReset(route, ref race, ref iso);
            }
        }

        /// <summary>Loads track + vehicle files (the reference lane).</summary>
        public static Sim Load(string trackPath, string vehiclePath, SimOptions options)
        {
            TrackLoader.TmnfTrack track = TrackLoader.TmnfTrackLoad(trackPath, null);
            byte[] vehicleBlob;
            try
            {
                vehicleBlob = System.IO.File.ReadAllBytes(vehiclePath);
            }
            catch (Exception)
            {
                throw new InvalidOperationException("cannot read vehicle snapshot");
            }
            string maskPath = options.fakeContactMask ?? "local/game-mask.bin";
            byte[] mask;
            try
            {
                mask = System.IO.File.ReadAllBytes(maskPath);
            }
            catch (Exception)
            {
                throw new InvalidOperationException("fake-contact mask (see SimOptions.fakeContactMask)");
            }
            return new Sim(track, vehicleBlob, mask, options);
        }

        /// <summary>One 10 ms tick: input, physics, race, observation.</summary>
        public VehicleState Step(ref TmnfInput input)
        {
            TMNFRaceInputs packet = input.ToPacket(tick);

            // The respawn press applies before the tick's control mapping.
            if (packet.respawn != 0)
            {
                GmIso4? spawn = RaceLayer.TmnfRaceRespawnLocation(ref race);
                if (spawn.HasValue)
                {
                    GmIso4 s = spawn.Value;
                    world.Respawn(ref s);
                }
            }
            world.route = route;
            world.ApplyInputs(ref packet);
            tick = unchecked(tick + 1);
            world.AdvanceTimer(10);
            world.PhysicsStep2(10);
            if (route != null && race.finished == 0)
            {
                GmIso4 iso = world.Live().RotPosIso();
                ulong contacts = world.triggerContacts;
                RaceLayer.TmnfRaceStep(route, ref race, contacts, ref iso);
            }
            return State();
        }

        /// <summary>Convenience overload (struct copy).</summary>
        public VehicleState Step(TmnfInput input)
        {
            TmnfInput copy = input;
            return Step(ref copy);
        }

        /// <summary>The observation of the current state (without stepping).</summary>
        public VehicleState State()
        {
            ref CHmsStateDyna s = ref world.Live();
            // The dyna quaternion bytes are (w, x, y, z); expose (x, y, z, w).
            float[] q = { s.quat.y, s.quat.z, s.quat.w, s.quat.x };
            VehicleState state = VehicleState.Create();
            state.raceTimeMs = unchecked(race.elapsedTicks * 10);
            state.position = s.pos;
            state.quaternion = q;
            state.linearSpeed = s.linVel;
            state.angularSpeed = s.angVel;
            for (int i = 0; i < 4; i++)
            {
                CSceneVehicleCarWheelRealTimeState rt = world.vehicle.wheels[i].realTime;
                state.wheels[i].onGround = rt.hasGroundContact != 0;
                state.wheels[i].contactMaterialId = rt.contactMaterialId;
                state.wheels[i].sliding = rt.isSliding != 0;
                state.wheels[i].spinSpeed = rt.field6c;
            }
            state.engineRpm = world.vehicle.engine.rpm;
            state.gear = world.vehicle.engine.gear;
            state.canRespawn = RaceLayer.TmnfRaceRespawnLocation(ref race).HasValue;
            return state;
        }

        /// <summary>Access to the underlying world (advanced use).</summary>
        public World World => world;

        /// <summary>The race state, if a route is bound.</summary>
        public RaceLayer.TmnfRaceState? Race()
        {
            if (route != null)
            {
                return race;
            }
            return null;
        }
    }
}
