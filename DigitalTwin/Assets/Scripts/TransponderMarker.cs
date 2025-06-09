using UnityEngine;

[RequireComponent(typeof(LineRenderer))]
public class TransponderLine : MonoBehaviour
{
    public Transform rov;
    public Material lineMaterial;
    public GameObject beamParticlesPrefab;

    private LineRenderer lineRenderer;
    private GameObject beamParticles;

void Start()
{
    // Set up LineRenderer
    lineRenderer = GetComponent<LineRenderer>();

    if (lineMaterial != null)
    {
        // Create a new instance of the material for this line
        lineRenderer.material = new Material(lineMaterial);
    }
    else
    {
        // Use a basic Unlit material if none assigned
        lineRenderer.material = new Material(Shader.Find("Unlit/Color"));
        lineRenderer.material.color = Color.cyan;
    }

    // Enable emission glow
    lineRenderer.material.EnableKeyword("_EMISSION");
    lineRenderer.material.SetColor("_EmissionColor", Color.cyan * 2f);

    // Line settings
    lineRenderer.startColor = Color.cyan;
    lineRenderer.endColor = Color.cyan;
    lineRenderer.startWidth = 0.005f;
    lineRenderer.endWidth = 0.005f;
    lineRenderer.positionCount = 2;

    // Instantiate particle beam (not parented)
    if (beamParticlesPrefab != null)
    {
        beamParticles = Instantiate(beamParticlesPrefab);
    }
}

void Update()
{
    if (rov == null)
        return;

    Vector3 start = rov.position;
    Vector3 end = transform.position;

    // Update line positions
    lineRenderer.SetPosition(0, start);
    lineRenderer.SetPosition(1, end);

    // Pulse the emission to make it glow
    float emissionStrength = Mathf.PingPong(Time.time * 8f, 1f) + 1f;
    lineRenderer.material.SetColor("_EmissionColor", Color.cyan * emissionStrength);

    // Update particle beam position and rotation
    if (beamParticles != null)
    {
        Vector3 direction = (end - start).normalized;

        beamParticles.transform.position = start;
        beamParticles.transform.rotation = Quaternion.LookRotation(direction) * Quaternion.Euler(90, 0, 0);

        var main = beamParticles.GetComponent<ParticleSystem>().main;
        main.startSpeed = (end - start).magnitude;
    }
}
}