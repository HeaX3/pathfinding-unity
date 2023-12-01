using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace com.heax3.pathfinding_unity
{
    public static class VisualUtil 
    {
        public static LineRenderer GeneratePathLine(Color color)
        {
            GameObject go = new GameObject();
            LineRenderer trianglelineRenderer = go.AddComponent<LineRenderer>();
            trianglelineRenderer.material = new Material(Shader.Find("Sprites/Default"));
            trianglelineRenderer.widthMultiplier = 0.3f;

            float alpha = 1.0f;
            Gradient gradient = new Gradient();
            gradient.SetKeys(
                new GradientColorKey[] { new GradientColorKey(color, 0.0f), new GradientColorKey(color, 1.0f) },
                new GradientAlphaKey[] { new GradientAlphaKey(alpha, 0.0f), new GradientAlphaKey(alpha, 1.0f) }
            );
            trianglelineRenderer.colorGradient = gradient;

            return trianglelineRenderer;
        }
    }
}
