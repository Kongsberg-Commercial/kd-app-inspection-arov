using UnityEngine;

public class CameraFollowScript : MonoBehaviour
{
    [Header("Follow Settings")]
    [Tooltip("The object that will move (e.g. your Camera)")]
    public Transform follower;

    [Tooltip("The object to follow on the X axis")]
    public Transform followed;

    // internal state
    private float _offsetX;
    private bool  _initialized;

    void Update()
    {
        if (follower == null || followed == null)
            return;

        // on first frame, capture the offset between follower and followed
        if (!_initialized)
        {
            _offsetX = follower.position.x - followed.position.x;
            _initialized = true;
        }

        // apply the same delta movement along X
        Vector3 pos = follower.position;
        pos.x = followed.position.x + _offsetX;
        follower.position = pos;
    }
}
