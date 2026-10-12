using Tmnf.Sim;
using Tmnf.Sim.Unity;
using UnityEngine;

namespace Tmnf.Game
{
    /// <summary>
    /// Runtime material helper: URP-first shader lookup with sensible
    /// fallbacks so placeholder geometry renders under any pipeline the
    /// project may switch to.
    /// </summary>
    public static class GameMaterials
    {
        private static Shader litShader;

        public static Shader LitShader()
        {
            if (litShader == null)
            {
                litShader = Shader.Find("Universal Render Pipeline/Lit");
                if (litShader == null)
                {
                    litShader = Shader.Find("Standard");
                }
            }
            return litShader;
        }

        public static Material Lit(Color color, float smoothness = 0.35f)
        {
            Shader shader = LitShader();
            if (shader == null)
            {
                Debug.LogWarning("[Tmnf.Game] no lit shader found; using a default material.");
                return new Material(Shader.Find("Sprites/Default")) { color = color };
            }
            Material material = new Material(shader);
            if (material.HasProperty("_BaseColor"))
            {
                material.SetColor("_BaseColor", color);
            }
            if (material.HasProperty("_Color"))
            {
                material.SetColor("_Color", color);
            }
            if (material.HasProperty("_Smoothness"))
            {
                material.SetFloat("_Smoothness", smoothness);
            }
            if (material.HasProperty("_Glossiness"))
            {
                material.SetFloat("_Glossiness", smoothness);
            }
            return material;
        }

        /// <summary>A material for GL line drawing (debug visualizer).</summary>
        public static Material LineMaterial()
        {
            Shader shader = Shader.Find("Hidden/Internal-Colored");
            if (shader == null)
            {
                shader = Shader.Find("Sprites/Default");
            }
            if (shader == null)
            {
                shader = Shader.Find("Universal Render Pipeline/Particles/Unlit");
            }
            if (shader == null)
            {
                return null;
            }
            Material material = new Material(shader)
            {
                hideFlags = HideFlags.HideAndDontSave,
            };
            material.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
            material.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
            material.SetInt("_Cull", (int)UnityEngine.Rendering.CullMode.Off);
            material.SetInt("_ZWrite", 0);
            return material;
        }

        /// <summary>An unlit material for UI-style overlays (countdown text etc.).</summary>
        public static Material Unlit(Color color)
        {
            Shader shader = Shader.Find("Universal Render Pipeline/Unlit");
            if (shader == null)
            {
                shader = Shader.Find("Unlit/Color");
            }
            if (shader == null)
            {
                return Lit(color, 0.0f);
            }
            Material material = new Material(shader);
            if (material.HasProperty("_BaseColor"))
            {
                material.SetColor("_BaseColor", color);
            }
            if (material.HasProperty("_Color"))
            {
                material.SetColor("_Color", color);
            }
            return material;
        }
    }
}
