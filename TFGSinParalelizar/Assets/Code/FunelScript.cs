using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FunelScript : MonoBehaviour
{
    // Start is called before the first frame update

    private List<Collider> colliders = new List<Collider>();
    public List<Collider> GetColliders() { return colliders; }
    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void OnTriggerEnter(Collider other)
    {
        if (!colliders.Contains(other)) { colliders.Add(other); }
    }

    private void OnTriggerExit(Collider other)
    {
        colliders.Remove(other);
    }
}
