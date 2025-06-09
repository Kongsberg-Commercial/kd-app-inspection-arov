using UnityEngine;
using System.Collections;

public class FishSpawner : MonoBehaviour
{
    [Header("Spawner Settings")]
    public GameObject fishPrefab;            // Your fish prefab.
    public int numberOfFish = 50;              // Total number of fish to spawn.
    public Vector3 spawnAreaCenter = Vector3.zero;   // Center of the spawn area.
    public Vector3 spawnAreaSize = new Vector3(50, 10, 50); // Dimensions of the spawn area.
    public float spawnDelay = 0.05f;           // Delay between each fish spawn (in seconds).

    void Start()
    {
        StartCoroutine(SpawnFishCoroutine());
    }

    IEnumerator SpawnFishCoroutine()
    {
        for (int i = 0; i < numberOfFish; i++)
        {
            // Calculate a random position within the spawn area.
            Vector3 randomPosition = spawnAreaCenter + new Vector3(
                Random.Range(-spawnAreaSize.x / 2f, spawnAreaSize.x / 2f),
                Random.Range(-spawnAreaSize.y / 2f, spawnAreaSize.y / 2f),
                Random.Range(-spawnAreaSize.z / 2f, spawnAreaSize.z / 2f)
            );

            // Optionally, choose a random rotation around the Y axis.
            Quaternion randomRotation = Quaternion.Euler(0, Random.Range(0, 360), 0);

            // Instantiate the fish prefab as a child of the spawner.
            Instantiate(fishPrefab, randomPosition, randomRotation, transform);

            // Wait a little before spawning the next fish.
            yield return new WaitForSeconds(spawnDelay);
        }
    }
}
