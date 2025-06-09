using UnityEngine;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class TrapezoidBeam : MonoBehaviour
{
    // Exposed parameters that you can tweak from the Inspector.
    [Header("Trapezoid Dimensions")]
    [Tooltip("Total width of the top edge of the trapezoid.")]
    public float topWidth = 0.4f;  // Total width at the top

    [Tooltip("Total width of the bottom edge of the trapezoid.")]
    public float bottomWidth = 1f; // Total width at the bottom

    [Tooltip("Height of the trapezoid (vertical distance from bottom to top).")]
    public float height = 1f;      // Vertical height of the trapezoid


    // If you want to see changes in the editor immediately, use OnValidate
    private void OnValidate()
    {
        MeshFilter meshFilter = GetComponent<MeshFilter>();
        if (meshFilter != null)
        {
            meshFilter.sharedMesh = CreateTrapezoidMesh();

        }
    }


    private void Start()
    {
        MeshFilter meshFilter = GetComponent<MeshFilter>();
        meshFilter.mesh = CreateTrapezoidMesh();
    }
    
    Mesh CreateTrapezoidMesh()
    {
        Mesh mesh = new Mesh();

        // Calculate half widths for the top and bottom.
        float halfTop = topWidth / 2f;
        float halfBottom = bottomWidth / 2f;
        
        // Define vertices for a trapezoid:
        // Vertices are ordered (clockwise or counter-clockwise) for the triangle indices
        Vector3[] vertices = new Vector3[4];
        vertices[0] = new Vector3(-halfTop, height, 0f);  // Top left (narrow)
        vertices[1] = new Vector3(halfTop, height, 0f);   // Top right (narrow)
        vertices[2] = new Vector3(-halfBottom, 0f, 0f);     // Bottom left (wide)
        vertices[3] = new Vector3(halfBottom, 0f, 0f);      // Bottom right (wide)
        
        // UV coordinates for texture mapping
        Vector2[] uv = new Vector2[4];
        uv[0] = new Vector2(0, 1);
        uv[1] = new Vector2(1, 1);
        uv[2] = new Vector2(0, 0);
        uv[3] = new Vector2(1, 0);
        
        // Define triangles (two triangles form a quad).
        int[] triangles = new int[6] 
        {
            0, 2, 1,   // First triangle: Top left, Bottom left, Top right
            1, 2, 3    // Second triangle: Top right, Bottom left, Bottom right
        };
        
        mesh.vertices = vertices;
        mesh.uv = uv;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();
        
        return mesh;
    }
}
