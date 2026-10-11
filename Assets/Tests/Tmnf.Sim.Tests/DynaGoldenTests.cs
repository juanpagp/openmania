using System;
using System;
using NUnit.Framework;
using Tmnf.Sim;

namespace Tmnf.Sim.Tests
{
    /// <summary>Golden suites for the dyna module (rigid-body integrator).</summary>
    [TestFixture]
    public class DynaGoldenTests
    {
        private static CHmsDynaParams ReadParams(ByteReader r)
        {
            byte[] b = r.Bytes(0x5c);
            return CHmsDynaParams.FromBytes(b);
        }

        private static CHmsStateDyna ReadState(ByteReader r)
        {
            byte[] b = r.Bytes(180);
            return CHmsStateDyna.FromBytes(b);
        }

        private static byte[] DumpDyna(CHmsDyna d)
        {
            var o = new System.Collections.Generic.List<byte>(256);
            o.AddRange(d.@params.ToBytes());
            o.AddRange(d.Live().ToBytes());
            o.AddRange(d.StateB().ToBytes());
            o.AddRange(d.tempState.ToBytes());
            ByteWriter.I32(o, d.clampAngular);
            ByteWriter.F32(o, d.maxAngularSpeed);
            ByteWriter.I32(o, d.dirtyFlag);
            ByteWriter.I32(o, d.mode);
            ByteWriter.U32(o, (uint)d.replacementBuf.Count);
            foreach (GmVec3 v in d.replacementBuf)
            {
                GmGoldenTests.WriteVec3To(o, v);
            }
            return o.ToArray();
        }

        [Test]
        public void IntegrateStep()
        {
            GoldenRunner.RunSuite("dyna/integrate_step.tgn", input =>
            {
                var r = new ByteReader(input);
                r.U32(); // leading (unused) mass bits word
                CHmsDynaParams parameters = ReadParams(r);
                int mode = r.I32();
                int clamp = r.I32();
                float maxAng = r.F32();
                float dt = r.F32();
                CHmsStateDyna state = ReadState(r);
                CHmsDyna dyna = new CHmsDyna(parameters, mode, clamp, maxAng);
                dyna.SetLive(state);
                dyna.SetStateB(state);
                CHmsStateDyna output = state;
                dyna.IntegrateStep(ref state, ref output, dt);
                return output.ToBytes();
            });
        }

        [Test]
        public void AddImpulse()
        {
            GoldenRunner.RunSuite("dyna/add_impulse.tgn", input =>
            {
                var r = new ByteReader(input);
                CHmsDynaParams parameters = ReadParams(r);
                int mode = r.I32();
                CHmsStateDyna state = ReadState(r);
                GmVec3 j = GmGoldenTests.ReadVec3(r);
                CHmsDyna dyna = new CHmsDyna(parameters, mode, 0, 0.0f);
                dyna.SetLive(state);
                dyna.AddImpulse(ref j);
                var o = new System.Collections.Generic.List<byte>();
                o.AddRange(dyna.Live().ToBytes());
                ByteWriter.I32(o, dyna.dirtyFlag);
                return o.ToArray();
            });
        }

        [Test]
        public void AddImpulseAt()
        {
            GoldenRunner.RunSuite("dyna/add_impulse_at.tgn", input =>
            {
                var r = new ByteReader(input);
                CHmsDynaParams parameters = ReadParams(r);
                int mode = r.I32();
                CHmsStateDyna state = ReadState(r);
                GmVec3 j = GmGoldenTests.ReadVec3(r);
                GmVec3 at = GmGoldenTests.ReadVec3(r);
                CHmsDyna dyna = new CHmsDyna(parameters, mode, 0, 0.0f);
                dyna.SetLive(state);
                dyna.AddImpulseAt(ref j, ref at);
                var o = new System.Collections.Generic.List<byte>();
                o.AddRange(dyna.Live().ToBytes());
                ByteWriter.I32(o, dyna.dirtyFlag);
                return o.ToArray();
            });
        }

        [Test]
        public void GetSpeed()
        {
            GoldenRunner.RunSuite("dyna/get_speed.tgn", input =>
            {
                var r = new ByteReader(input);
                CHmsDynaParams parameters = ReadParams(r);
                int mode = r.I32();
                CHmsStateDyna state = ReadState(r);
                GmVec3 at = GmGoldenTests.ReadVec3(r);
                CHmsDyna dyna = new CHmsDyna(parameters, mode, 0, 0.0f);
                GmVec3 speed = dyna.GetSpeed(ref at);
                return GmGoldenTests.WriteVec3(speed);
            });
        }

        [Test]
        public void Replacements()
        {
            GoldenRunner.RunSuite("dyna/replacements.tgn", input =>
            {
                var r = new ByteReader(input);
                CHmsDynaParams parameters = ReadParams(r);
                int mode = r.I32();
                CHmsStateDyna state = ReadState(r);
                GmVec3 d = GmGoldenTests.ReadVec3(r);
                uint replCount = r.U32();
                var repls = new GmVec3[replCount];
                for (int i = 0; i < (int)replCount; i++)
                {
                    repls[i] = GmGoldenTests.ReadVec3(r);
                }
                CHmsDyna dyna = new CHmsDyna(parameters, mode, 0, 0.0f);
                dyna.SetLive(state);
                dyna.ApplyReplacement(ref d);
                foreach (GmVec3 rep in repls)
                {
                    GmVec3 copy = rep;
                    dyna.AddReplacement(ref copy);
                }
                GmVec3 synth = dyna.ComputeSynthetizedReplacement();
                var o = new System.Collections.Generic.List<byte>();
                GmGoldenTests.WriteVec3To(o, synth);
                dyna.DoPostCollisionDynamic();
                o.AddRange(dyna.Live().ToBytes());
                return o.ToArray();
            });
        }

        [Test]
        public void StateRoundTrip()
        {
            GoldenRunner.RunSuite("dyna/state_roundtrip.tgn", input =>
            {
                byte[] bytes = input;
                CHmsStateDyna back = CHmsStateDyna.FromBytes(bytes);
                return back.ToBytes();
            });
        }

        [Test]
        public void PreCollision()
        {
            GoldenRunner.RunSuite("dyna/pre_collision.tgn", input =>
            {
                var r = new ByteReader(input);
                CHmsDynaParams parameters = ReadParams(r);
                int mode = r.I32();
                int clamp = r.I32();
                float maxAng = r.F32();
                float dt = r.F32();
                CHmsStateDyna state = ReadState(r);
                uint replCount = r.U32();
                CHmsDyna dyna = new CHmsDyna(parameters, mode, clamp, maxAng);
                dyna.SetLive(state);
                for (int i = 0; i < (int)replCount; i++)
                {
                    GmVec3 rep = GmGoldenTests.ReadVec3(r);
                    dyna.AddReplacement(ref rep);
                }
                dyna.DoPreCollisionDynamic(dt);
                return dyna.Live().ToBytes();
            });
        }
    }
}
