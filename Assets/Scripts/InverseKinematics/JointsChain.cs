using System.Linq;
using UnityEngine;

namespace Fusiology.InverseKinematics
{
    /// <summary>
    /// Represents a chain of multiple joints whose position and rotation is automatically updated with inverse kinematics.
    /// It implements the FABRIK algorithm which determines the positions of an interconnected joints set.
    /// cf http://andreasaristidou.com/FABRIK.html
    /// </summary>
    /// <seealso cref="UnityEngine.MonoBehaviour" />
    public class JointsChain : MonoBehaviour
    {
        #region Private inspector fields

        /// <summary>
        /// The set of the interconnected joints.
        /// They are sorted by hierarchy, from the fixed origin joint (first <see cref="Transform"/>) to the end joint (last <see cref="Transform"/>).
        /// The array must contain at least 2 items.
        /// </summary>
        [SerializeField]
        private Transform[] m_Joints;

        /// <summary>
        /// <see cref="Target"/> backing field.
        /// </summary>
        [SerializeField]
        private Vector3 m_Target;

        /// <summary>
        /// The tolerance to use in the FABRIK algorithm.
        /// It corresponds to the distance tolerance between <see cref="m_Target"/> and the end joint bellow which the FABRIK algorithm stops.
        /// </summary>
        [SerializeField]
        [Min(Vector3.kEpsilon)]
        private float m_Tolerance = 0.01f;  // TODO: auto compute tolerance (from a % of the total joints length)

        #endregion Private inspector fields

        #region Private fields

        /// <summary>
        /// The joints positions array indexed by the joint ID which is preallocated to avoid multiple allocations in <see cref="UpdateChain"/>.
        /// </summary>
        private Vector3[] m_JointsPositions;

        /// <summary>
        /// The inter-joint distances array which is preallocated to avoid multiple allocations in <see cref="UpdateChain"/>.
        /// The element of index i is the distance between the joint i and i + 1 in the <see cref="m_Joints"/> array.
        /// </summary>
        private float[] m_InterJointDistances;

        /// <summary>
        /// The sum of all the inter-joint distances.
        /// </summary>
        private float m_InterJointTotalDistance;

        #endregion Private fields

        #region Properties

        /// <summary>
        /// The target <see cref="Vector3"/> on which the end joint should be attached to.
        /// </summary>
        public Vector3 Target
        {
            get => m_Target;
            set => m_Target = value;
        }

        #endregion Properties

        #region Life-cycle methods

        /// <summary>
        /// Set up the FABRIK algorithm.
        /// </summary>
        private void OnEnable() => SetUp();

        /// <summary>
        /// Run the FABRIK algorithm each frame when this component is enabled.
        /// </summary>
        private void Update() => UpdateChain();

        #endregion Life-cycle methods

        #region FABRIK methods

        /// <summary>
        /// Set up the FABRIK algorithm by preallocating some arrays.
        /// It must be invoked each time a new joint is inserted into <see cref="m_Joints"/> or when the distance between joints is changed.
        /// </summary>
        [ContextMenu(nameof(SetUp))]
        private void SetUp()
        {
            // The inverse kinematics can not be performed if the chain does not contain at least 2 joints.
            if (m_Joints.Length < 2)
            {
                Debug.LogWarning($"{nameof(JointsChain)}: the joints chain must contain at least 2 joints.");
                enabled = false;
                return;
            }

            if (m_JointsPositions == null || m_JointsPositions.Length != m_Joints.Length)
            {
                m_JointsPositions = new Vector3[m_Joints.Length];
            }

            if (m_InterJointDistances == null || m_InterJointDistances.Length != m_Joints.Length - 1)
            {
                m_InterJointDistances = new float[m_Joints.Length - 1];
            }

            for (int i = 0; i < m_InterJointDistances.Length; i++)
            {
                m_InterJointDistances[i] = Vector3.Distance(m_Joints[i].position, m_Joints[i + 1].position);
            }

            m_InterJointTotalDistance = m_InterJointDistances.Sum();
        }

        /// <summary>
        /// Run the FABRIK algorithm which updates the positions of all the <see cref="m_Joints"/> in order to move the end joint on the <see cref="m_Target"/>.
        /// </summary>
        [ContextMenu(nameof(UpdateChain))]
        private void UpdateChain()  // TODO: improve loop index management
        {
            // Get the current joints transforms positions.
            for (int i = 0; i < m_Joints.Length; i++)
            {
                m_JointsPositions[i] = m_Joints[i].position;
            }

            // Check if target is not within reach.
            if (m_InterJointTotalDistance < Vector3.Distance(m_JointsPositions[0], m_Target))  // TODO: use squared distance ?
            {
                for (int i = 0; i < m_InterJointDistances.Length; i++)
                {
                    float segmentProportion = m_InterJointDistances[i] / Vector3.Distance(m_Target, m_JointsPositions[i]);
                    m_JointsPositions[i + 1] = (1 - segmentProportion) * m_JointsPositions[i] + segmentProportion * m_Target;
                }
            }
            else
            {
                // Target is reachable, start the forward and backward reaching iterations.
                Vector3 initialPosition = m_JointsPositions[0];
                int lastJointIndex = m_JointsPositions.Length - 1;

                while (Vector3.Distance(m_JointsPositions[lastJointIndex], m_Target) > m_Tolerance)    // TODO: fix infinite loop
                {
                    // Forward reaching: set the last joint on the target and propagate the movement to the other joints.
                    m_JointsPositions[lastJointIndex] = m_Target;

                    for (int i = lastJointIndex - 1; i >= 0; i--)
                    {
                        float segmentProportion = m_InterJointDistances[i] / Vector3.Distance(m_JointsPositions[i + 1], m_JointsPositions[i]);
                        m_JointsPositions[i] = (1 - segmentProportion) * m_JointsPositions[i + 1] + segmentProportion * m_JointsPositions[i];
                    }

                    // Backward reaching: set the first joint on the initial position and propagate the movement to the other joints.
                    m_JointsPositions[0] = initialPosition;

                    for (int i = 0; i < lastJointIndex; i++)
                    {
                        float segmentProportion = m_InterJointDistances[i] / Vector3.Distance(m_JointsPositions[i + 1], m_JointsPositions[i]);
                        m_JointsPositions[i + 1] = (1 - segmentProportion) * m_JointsPositions[i] + segmentProportion * m_JointsPositions[i + 1];
                    }
                }
            }

            // Apply the new joints positions and rotations except for the origin joint which always remains at the same position.
            m_Joints[0].rotation = Quaternion.FromToRotation(Vector3.forward, m_JointsPositions[1] - m_JointsPositions[0]);     // TODO: optimize by using m_Joints[i].forward instead if Vector3.forward
            for (int i = 1; i < m_Joints.Length - 1; i++)
            {
                m_Joints[i].SetPositionAndRotation(m_JointsPositions[i], Quaternion.FromToRotation(Vector3.forward, m_JointsPositions[i + 1] - m_JointsPositions[i]));
            }
            m_Joints[m_Joints.Length - 1].position = m_JointsPositions[m_JointsPositions.Length - 1];
        }

        #endregion FABRIK methods
    }
}