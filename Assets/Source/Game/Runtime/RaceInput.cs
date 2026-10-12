using Tmnf.Sim;
using UnityEngine;
using UnityEngine.InputSystem;

namespace Tmnf.Game
{
    /// <summary>
    /// Keyboard input for the race, sampled once per rendered frame and fed
    /// to the simulation's input packet (the same digital model the golden
    /// traces use). Also exposes the one-shot actions (reset, pause, debug
    /// toggles) as edge-triggered properties.
    ///
    /// Default mapping (Input System package; the project's active input
    /// handler is set to it):
    ///   W / Up      accelerate
    ///   S / Down    brake / reverse
    ///   A / Left    steer left
    ///   D / Right   steer right
    ///   R / Backspace  restart the run from the start
    ///   Escape / P     pause menu
    ///   F3          collision debug rendering
    ///   V           toggle the visible car model
    /// </summary>
    public sealed class RaceInput
    {
        /// <summary>The driving input for this frame.</summary>
        public TmnfInput Drive;

        /// <summary>Restart-run key pressed this frame.</summary>
        public bool ResetPressed;

        /// <summary>Pause key pressed this frame.</summary>
        public bool PausePressed;

        /// <summary>Collision-debug key pressed this frame.</summary>
        public bool DebugPressed;

        /// <summary>Vehicle-visuals key pressed this frame.</summary>
        public bool VisualsPressed;

        /// <summary>Reads the keyboard. Safe with no keyboard attached.</summary>
        public void Sample()
        {
            Drive = default(TmnfInput);
            ResetPressed = false;
            PausePressed = false;
            DebugPressed = false;
            VisualsPressed = false;

            Keyboard keyboard = Keyboard.current;
            if (keyboard == null)
            {
                return;
            }
            Drive.accelerate = keyboard.wKey.isPressed || keyboard.upArrowKey.isPressed;
            Drive.brake = keyboard.sKey.isPressed || keyboard.downArrowKey.isPressed;
            Drive.steerLeft = keyboard.aKey.isPressed || keyboard.leftArrowKey.isPressed;
            Drive.steerRight = keyboard.dKey.isPressed || keyboard.rightArrowKey.isPressed;
            ResetPressed = keyboard.rKey.wasPressedThisFrame
                || keyboard.backspaceKey.wasPressedThisFrame;
            PausePressed = keyboard.escapeKey.wasPressedThisFrame
                || keyboard.pKey.wasPressedThisFrame;
            DebugPressed = keyboard.f3Key.wasPressedThisFrame;
            VisualsPressed = keyboard.vKey.wasPressedThisFrame;
        }
    }
}
