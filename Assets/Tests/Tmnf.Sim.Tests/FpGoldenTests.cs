using System;
using NUnit.Framework;
using Tmnf.Sim;

namespace Tmnf.Sim.Tests
{
    /// <summary>Golden suites for the fp module (the FP policy itself).</summary>
    [TestFixture]
    public class FpGoldenTests
    {
        [Test]
        public void Ftol()
        {
            int rows = GoldenRunner.RunSuite("fp/ftol.tgn", input =>
            {
                var r = new ByteReader(input);
                double x = r.F64();
                return BitConverter.GetBytes(Fp.Ftol(x));
            });
            Assert.That(rows, Is.GreaterThan(4000), "row count");
        }

        [Test]
        public void U32ToX87Float()
        {
            int rows = GoldenRunner.RunSuite("fp/u32_to_x87_float.tgn", input =>
            {
                var r = new ByteReader(input);
                uint v = r.U32();
                return BitConverter.GetBytes(Fp.U32ToX87Float(v));
            });
            Assert.That(rows, Is.GreaterThan(5000), "row count");
        }

        [Test]
        public void X87Sin()
        {
            RunUnary("fp/x87_sin.tgn", Fp.X87Sin);
        }

        [Test]
        public void X87Cos()
        {
            RunUnary("fp/x87_cos.tgn", Fp.X87Cos);
        }

        [Test]
        public void X87Exp()
        {
            RunUnary("fp/x87_exp.tgn", Fp.X87Exp);
        }

        [Test]
        public void X87Atan2()
        {
            int rows = GoldenRunner.RunSuite("fp/x87_atan2.tgn", input =>
            {
                var r = new ByteReader(input);
                float y = r.F32();
                float x = r.F32();
                return BitConverter.GetBytes(Fp.X87Atan2(y, x));
            });
            Assert.That(rows, Is.GreaterThan(100_000), "row count");
        }

        [Test]
        public void X87Fmod()
        {
            int rows = GoldenRunner.RunSuite("fp/x87_fmod.tgn", input =>
            {
                var r = new ByteReader(input);
                float v = r.F32();
                float p = r.F32();
                return BitConverter.GetBytes(Fp.X87Fmod(v, p));
            });
            Assert.That(rows, Is.GreaterThan(10_000), "row count");
        }

        private static void RunUnary(string path, System.Func<float, float> f)
        {
            int rows = GoldenRunner.RunSuite(path, input =>
            {
                var r = new ByteReader(input);
                float x = r.F32();
                return BitConverter.GetBytes(f(x));
            });
            Assert.That(rows, Is.GreaterThan(100_000), "row count");
        }
    }
}
