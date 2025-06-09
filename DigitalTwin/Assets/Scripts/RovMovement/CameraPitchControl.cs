using UnityEngine;
using UnityEngine.InputSystem;

public class CameraPitchControl : MonoBehaviour
{
    public float pitchSpeed = 60f;      // Degrees per second at full input
    public float minPitch = -60f;      // Lowest angle (looking down)
    public float maxPitch = 60f;       // Highest angle (looking up)

    private ROVInputActions inputActions;
    private float pitchInput;          // Current input from -1..1
    private float currentPitch;        // Current pitch angle

    private void Awake()
    {
        inputActions = new ROVInputActions();

        // Listen for pitch input
        inputActions.Player.CameraPitch.performed += ctx => pitchInput = ctx.ReadValue<float>();
        inputActions.Player.CameraPitch.canceled += ctx => pitchInput = 0f;
    }

    private void OnEnable()  => inputActions.Enable();
    private void OnDisable() => inputActions.Disable();

    private void Update()
    {
        // Update pitch angle based on input
        currentPitch += pitchInput * pitchSpeed * Time.deltaTime;

        // Clamp to avoid flipping the camera
        currentPitch = Mathf.Clamp(currentPitch, minPitch, maxPitch);

        // Apply pitch (local rotation around X-axis)
        Vector3 localEuler = transform.localEulerAngles;
        // Because localEulerAngles.x is 0..360, we convert to a -180..180 range if needed:
        localEuler.x = currentPitch;
        transform.localEulerAngles = localEuler;
    }
}
