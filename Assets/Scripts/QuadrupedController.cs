using System.Collections;
using UnityEngine;

namespace Fusiology
{
    /// <summary>
    /// Script which manages the movements and behaviour of a quadruped being.
    /// </summary>
    public class QuadrupedController : MonoBehaviour
    {
        [SerializeField]
        private Transform m_VirtualTarget;

        [SerializeField]
        private Transform m_Target;

        [SerializeField]
        private float m_Distance = 1f;

        private Coroutine m_LegMovementCoroutine;

        private void Update()
        {
            if (Vector3.Distance(m_VirtualTarget.position, m_Target.position) > m_Distance && m_LegMovementCoroutine == null)
            {
                m_LegMovementCoroutine = StartCoroutine(LegMovementCoroutine(m_VirtualTarget.position));
            }
        }

        private IEnumerator LegMovementCoroutine(Vector3 destination)
        {
            Vector3 deltaMovement = (destination - m_Target.position) / 60;

            for (int i = 0; i < 60; i++)
            {
                m_Target.position += deltaMovement;
                yield return null;
            }

            m_LegMovementCoroutine = null;
        }
    }
}