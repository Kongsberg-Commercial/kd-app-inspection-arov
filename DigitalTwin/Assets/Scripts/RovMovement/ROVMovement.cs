using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(Rigidbody))]
public class ROVMovement : MonoBehaviour
{
    [Header("Movement Settings")]
    public float moveSpeed = 5f;          // Determines acceleration for translation
    public float rotationSpeed = 90f;     // Determines acceleration for rotation
    public float dragFactor = 2f;         // Linear drag: Higher value means quicker deceleration
    public float angularDragFactor = 2f;  // Angular drag: Higher value means quicker rotational deceleration

    // This flag controls whether the ROV can be controlled.
    // Set this to false (from another script, e.g., CameraSwitcher) to disable ROV movement.
    public static bool controlEnabled = true;

    private ROVInputActions inputActions;
    private Rigidbody rb;

    // Corrected 2D input: X = left/right, Y = forward/back
    private Vector2 moveInput;
    // Rotation input from keyboard/gamepad (raw input value)
    private float rotationInput;
    // Vertical input (up/down)
    private float verticalInput;

    // Persistent velocity for drifting (translation)
    private Vector3 currentVelocity;
    // Persistent angular velocity for rotational drifting (in degrees per second)
    private float currentAngularVelocity = 0f;

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        inputActions = new ROVInputActions();

        // LEFT-RIGHT-UP-DOWN (WASD or left stick)
        // Rotate input 90° clockwise so W becomes forward, S backward, A left, D right.
        inputActions.Player.LeftRightUpDownMove.performed += ctx =>
        {
            Vector2 raw = ctx.ReadValue<Vector2>();
            // 90° clockwise rotation: (x, y) -> (y, -x)
            moveInput = new Vector2(raw.y, -raw.x);
        };
        inputActions.Player.LeftRightUpDownMove.canceled += ctx => moveInput = Vector2.zero;

        // ROTATION (e.g., right stick X or keyboard left/right arrows)
        inputActions.Player.Rotation.performed += ctx => rotationInput = ctx.ReadValue<float>();
        inputActions.Player.Rotation.canceled += ctx => rotationInput = 0f;

        // VERTICAL (e.g., triggers or Q/E keys)
        inputActions.Player.Vertical.performed += ctx => verticalInput = ctx.ReadValue<float>();
        inputActions.Player.Vertical.canceled += ctx => verticalInput = 0f;
    }

    private void OnEnable()  => inputActions.Enable();
    private void OnDisable() => inputActions.Disable();

    private void FixedUpdate()
    {
        // If control is disabled (e.g., static camera is active), do not process movement.
        if (!controlEnabled)
            return;

        // ---- Translational Movement ----
        // Build the input direction in local space:
        // - moveInput.x: left/right
        // - moveInput.y: forward/back
        // - verticalInput: up/down
        Vector3 inputDirection = new Vector3(
            moveInput.x,    // left/right
            verticalInput,  // up/down
            moveInput.y     // forward/back
        );

        // Convert to world space and scale by moveSpeed to get acceleration
        Vector3 acceleration = transform.TransformDirection(inputDirection) * moveSpeed;

        // Update persistent velocity
        currentVelocity += acceleration * Time.fixedDeltaTime;

        // Apply linear drag to simulate water friction (drift deceleration)
        currentVelocity = Vector3.Lerp(currentVelocity, Vector3.zero, dragFactor * Time.fixedDeltaTime);

        // Move the ROV using the current velocity
        rb.MovePosition(rb.position + currentVelocity * Time.fixedDeltaTime);

        // ---- Rotational Movement ----
        // Update angular velocity based on input:
        currentAngularVelocity += rotationInput * rotationSpeed * Time.fixedDeltaTime;

        // Apply angular drag so the rotation slows down when no input is provided.
        currentAngularVelocity = Mathf.Lerp(currentAngularVelocity, 0f, angularDragFactor * Time.fixedDeltaTime);

        // Apply the rotation (yaw) around the Y-axis:
        if (Mathf.Abs(currentAngularVelocity) > 0.01f)
        {
            Quaternion deltaRotation = Quaternion.Euler(0f, currentAngularVelocity * Time.fixedDeltaTime, 0f);
            rb.MoveRotation(rb.rotation * deltaRotation);
        }
    }
}
