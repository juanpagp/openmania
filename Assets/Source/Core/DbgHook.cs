using System;

namespace Tmnf.Sim
{
    /// <summary>
    /// The env-gated divergence-hunting switch shared by the diagnostic
    /// hooks sprinkled through the port (force stages, wheel forces, drive
    /// net, steering angle, replacements, collision buffer). Set TMNF_DBG=1
    /// to make them print; the cached read keeps the disabled path to a
    /// single static field load. The hooks are non-behavioral: with the
    /// flag unset they add no observable effect, so the golden gate is
    /// unaffected.
    /// </summary>
    internal static class DbgHook
    {
        public static readonly bool Enabled =
            Environment.GetEnvironmentVariable("TMNF_DBG") != null;
    }
}
