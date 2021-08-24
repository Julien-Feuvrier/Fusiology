using System.Linq;
using UnityEngine;

namespace Fusiology.InverseKinematics
{
    public class Fabrik : MonoBehaviour
    {
        [SerializeField]
        private Transform[] m_Joints;

        [SerializeField]
        private Transform m_Target;

        [SerializeField]
        private float m_Tolerance = 0.001f;

        private Vector3[] m_JointsPositions;

        private void Update() => Run();

        private void Run()
        {
            if (m_JointsPositions == null || m_JointsPositions.Length != m_Joints.Length)
            {
                m_JointsPositions = new Vector3[m_Joints.Length];
            }

            for (int i = 0; i < m_Joints.Length; i++)
            {
                m_JointsPositions[i] = m_Joints[i].position;
            }

            float[] jointsDistances = new float[m_JointsPositions.Length - 1];
            float distance = Vector3.Distance(m_JointsPositions[0], m_Target.position);

            for (int i = 0; i < jointsDistances.Length; i++)
            {
                jointsDistances[i] = Vector3.Distance(m_JointsPositions[i], m_JointsPositions[i + 1]);
            }

            // Target is not within reach.
            if (jointsDistances.Sum() < distance)
            {
                for (int i = 0; i < jointsDistances.Length; i++)
                {
                    float ri = Vector3.Distance(m_Target.position, m_JointsPositions[i]);
                    float lambdai = jointsDistances[i] / ri;
                    m_JointsPositions[i + 1] = (1 - lambdai) * m_JointsPositions[i] + lambdai * m_Target.position;
                }
            }
            else
            {
                Vector3 initialPosition = m_JointsPositions[0];
                int lastJointIndex = m_JointsPositions.Length - 1;

                while (Vector3.Distance(m_JointsPositions[lastJointIndex], m_Target.position) > m_Tolerance)    // TODO: fix infinite loop
                {
                    // Forward reaching.
                    m_JointsPositions[lastJointIndex] = m_Target.position;

                    for (int i = lastJointIndex - 1; i >= 0; i--)
                    {
                        float ri = Vector3.Distance(m_JointsPositions[i + 1], m_JointsPositions[i]);
                        float lambdai = jointsDistances[i] / ri;
                        m_JointsPositions[i] = (1 - lambdai) * m_JointsPositions[i + 1] + lambdai * m_JointsPositions[i];
                    }

                    // Backward reaching.
                    m_JointsPositions[0] = initialPosition;

                    for (int i = 0; i < lastJointIndex; i++)
                    {
                        float ri = Vector3.Distance(m_JointsPositions[i + 1], m_JointsPositions[i]);
                        float lambdai = jointsDistances[i] / ri;
                        m_JointsPositions[i + 1] = (1 - lambdai) * m_JointsPositions[i] + lambdai * m_JointsPositions[i + 1];
                    }
                }
            }

            for (int i = 0; i < m_Joints.Length - 1; i++)
            {
                m_Joints[i].SetPositionAndRotation(m_JointsPositions[i], Quaternion.FromToRotation(Vector3.up, m_JointsPositions[i + 1] - m_JointsPositions[i]));
            }
        }
    }
}