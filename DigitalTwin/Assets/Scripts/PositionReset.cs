using UnityEngine;

public class ResetTransponderPositions : MonoBehaviour
{
    [Tooltip("Reference to the ROV transform. Its Z position will be used as the baseline.")]
    public Transform rov;

    [Tooltip("Assign only your transponder transforms here (do NOT include the ROV).")]
    public Transform[] transponders;

    [Tooltip("Optional: Uniform Z offset for all transponders relative to the ROV's Z position.")]
    public float zOffset = 0f;

    void Start()
    {
        if (rov == null)
        {
            Debug.LogError("ROV is not assigned!");
            return;
        }

        // Reset each transponder’s position so that X and Y are 0
        // and Z is set to the ROV's Z plus an optional offset.
        foreach (Transform t in transponders)
        {
            if (t != null)
            {
                Vector3 newPos = new Vector3(0f, 0f, rov.position.z + zOffset);
                t.position = newPos;
                Debug.Log($"{t.name} new position: {t.position}");
            }
        }
    }
}
