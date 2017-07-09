using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using CrispyPhysics;
public class Ball : MonoBehaviour {

    public float radius = 0.5f;
    public float mass = 1f;
    public float linearDamping = 0.01f;
    public float angularDamping = 0.2f;
    public float gravityScale = 1f;
    public float friction = 0.2f;
    public float restitution = 0.8f;

    public IBody body;
	
	// Update is called once per frame
	void Update () {
        transform.position = body.position;
        transform.rotation = Quaternion.Euler(0f, 0f, body.angle * Mathf.Rad2Deg);
    }

    public int lineCount = 100;

    static Material lineMaterial;
    static void CreateLineMaterial()
    {
        if (!lineMaterial)
        {
            // Unity has a built-in shader that is useful for drawing
            // simple colored things.
            Shader shader = Shader.Find("Hidden/Internal-Colored");
            lineMaterial = new Material(shader);
            lineMaterial.hideFlags = HideFlags.HideAndDontSave;
            // Turn on alpha blending
            //lineMaterial.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
            //lineMaterial.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
            // Turn backface culling off
            //lineMaterial.SetInt("_Cull", (int)UnityEngine.Rendering.CullMode.Off);
            // Turn off depth writes
            lineMaterial.SetInt("_ZWrite", 0);
        }
    }

    public void OnRenderObject()
    {
        SpriteRenderer spriteRenderer = GetComponent<SpriteRenderer>();
        Color color = spriteRenderer.color;
        CreateLineMaterial();
        // Apply the line material
        lineMaterial.SetPass(0);
        // Draw lines
        GL.Begin(GL.LINES);
        foreach (IMomentum momentum in body.MomentumIterator())
        {
            if (momentum.enduringContact == true)
                GL.Color(new Color(255, 255, 0, 0.8F));
            else if (momentum.tick <= body.current.tick)
                GL.Color(new Color(
                    spriteRenderer.color.r, 
                    spriteRenderer.color.g,
                    spriteRenderer.color.b, 0.5F
                ));
            else
                GL.Color(new Color(
                    spriteRenderer.color.r,
                    spriteRenderer.color.g,
                    spriteRenderer.color.b, 1F
                ));

            GL.Vertex3(momentum.position.x, momentum.position.y, 0);
        }
        GL.End();
    }
}
