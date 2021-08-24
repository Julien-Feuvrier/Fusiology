using System.Linq;
using UnityEngine;

namespace Fusiology.InverseKinematics
{
    /// <summary>
    /// Implementation of the FABRIK algorithm which determines the positions of an interconnected joints set through inverse kinematics.
    /// cf http://andreasaristidou.com/FABRIK.html
    /// </summary>
    /// <seealso cref="UnityEngine.MonoBehaviour" />
    public class Fabrik : MonoBehaviour
    {
        #region Private inspector fields

        /// <summary>
        /// The set of the interconnected joints.
        /// They are sorted by hierarchy, from the fixed origin joint (first <see cref="Transform"/>) to the edge joint (last <see cref="Transform"/>).
        /// </summary>
        [SerializeField]
        private Transform[] m_Joints;

        /// <summary>
        /// The target <see cref="Transform"/> on which the edge joint should be attached to.
        /// </summary>
        [SerializeField]
        private Transform m_Target;

        /// <summary>
        /// The tolerance to use in the FABRIK algorithm.
        /// It corresponds to the distance tolerance between <see cref="m_Target"/> and the edge joint bellow which the FABRIK algorithm stops.
        /// </summary>
        [SerializeField]
        private float m_Tolerance = 0.01f;

        #endregion Private inspector fields

        #region Private fields

        /// <summary>
        /// The joints positions array which is preallocated to avoid multiple allocations in <see cref="Run"/>.
        /// </summary>
        private Vector3[] m_JointsPositions;

        #endregion Private fields

        #region Life-cycle methods

        /// <summary>
        /// Run the FABRIK algorithm each frame when this component is enabled.
        /// </summary>
        private void Update() => Run();

        #endregion Life-cycle methods

        #region FABRIK methods

        /// <summary>
        /// Run the FABRIK algorithm which updates the positions of all the <see cref="m_Joints"/> in order to move the edge joint on the <see cref="m_Target"/>.
        /// </summary>
        [ContextMenu(nameof(Run))]
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

        #endregion FABRIK methods
    }
}