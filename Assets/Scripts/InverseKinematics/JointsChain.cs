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
        /// The joints rotations array indexed by the joint ID which is preallocated to avoid multiple allocations in <see cref="UpdateChain"/>.
        /// The rotations are defined in world space.
        /// </summary>
        private Quaternion[] m_JointsRotations;

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

            if (m_JointsRotations == null || m_JointsRotations.Length != m_Joints.Length)
            {
                m_JointsRotations = new Quaternion[m_Joints.Length];
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
        private void UpdateChain()
        {
            // Get the current joints transforms positions.
            for (int i = 0; i < m_Joints.Length; i++)
            {
                m_JointsPositions[i] = m_Joints[i].position;
            }

            // Check if target is not within reach.
            if (m_InterJointTotalDistance < Vector3.Distance(m_JointsPositions[0], m_Target))  // TODO: use squared distance for m_InterJointTotalDistance and for targetDistance
            {
                for (int i = 0; i < m_InterJointDistances.Length; i++)
                {
                    float segmentProportion = m_InterJointDistances[i] / Vector3.Distance(m_Target, m_JointsPositions[i]);
                    m_JointsPositions[i + 1] = (1 - segmentProportion) * m_JointsPositions[i] + segmentProportion * m_Target;
                }
            }
            else
            {
                // Set up the joints rotations array.
                for (int i = 0; i < m_JointsRotations.Length; i++)  // TODO: necessary to initialize this array ? or just use FromToRotation(Vector3.forward, m_JointsPositions[i + 1] - m_JointsPositions[i])
                {
                    m_JointsRotations[i] = m_Joints[i].rotation;
                }

                // Target is reachable, start the forward and backward reaching iterations.
                Vector3 initialPosition = m_JointsPositions[0];
                Quaternion initialRotation = m_JointsRotations[0];
                int lastJointIndex = m_JointsPositions.Length - 1;  // TODO: improve loop index management
                int iterationCount = 0;     // TODO: failsafe for debug, remove it in the future
                float targetDistance = Vector3.Distance(m_JointsPositions[lastJointIndex], m_Target);

                while (targetDistance > m_Tolerance && iterationCount++ < 30)    // TODO: fix infinite loop / all joints aligned issue by adding some randomness
                {
                    // FORWARD REACHING: set the last joint on the target and propagate the movement to the other joints.
                    m_JointsPositions[lastJointIndex] = m_Target;

                    // The penultimate joint position is manually set because constraints can not apply on the last 2 joints in forward reaching.
                    float penultimateSegmentProportion = m_InterJointDistances[lastJointIndex - 1] / Vector3.Distance(m_JointsPositions[lastJointIndex], m_JointsPositions[lastJointIndex - 1]);
                    m_JointsPositions[lastJointIndex - 1] = (1 - penultimateSegmentProportion) * m_JointsPositions[lastJointIndex] + penultimateSegmentProportion * m_JointsPositions[lastJointIndex - 1];

                    for (int i = lastJointIndex - 2; i >= 0; i--)
                    {
                        float segmentProportion = m_InterJointDistances[i] / Vector3.Distance(m_JointsPositions[i + 1], m_JointsPositions[i]);
                        m_JointsPositions[i] = (1 - segmentProportion) * m_JointsPositions[i + 1] + segmentProportion * m_JointsPositions[i];

                        // Correct position to satisfy the joint constraints.
                        Vector3 nextJointForward = m_JointsRotations[i + 1] * Vector3.forward;
                        Vector3 nextJointVector = m_JointsPositions[i + 2] - m_JointsPositions[i + 1];
                        float angle = Vector3.Angle(nextJointForward, nextJointVector);
                        if (angle > 45)
                        {
                            Quaternion correctionRotation = Quaternion.AngleAxis(angle - 45, Vector3.Cross(nextJointVector, nextJointForward));
                            m_JointsPositions[i] = m_JointsPositions[i + 1] + Quaternion.Inverse(correctionRotation) * (m_JointsPositions[i] - m_JointsPositions[i + 1]);
                        }
                    }

                    // BACKWARD REACHING: set the first joint on the initial position and propagate the movement to the other joints.
                    m_JointsPositions[0] = initialPosition;
                    m_JointsRotations[0] = initialRotation;

                    for (int i = 0; i < lastJointIndex; i++)
                    {
                        float segmentProportion = m_InterJointDistances[i] / Vector3.Distance(m_JointsPositions[i + 1], m_JointsPositions[i]);
                        m_JointsPositions[i + 1] = (1 - segmentProportion) * m_JointsPositions[i] + segmentProportion * m_JointsPositions[i + 1];

                        // Correct position to satisfy the joint constraints.
                        Vector3 jointForward = m_JointsRotations[i] * Vector3.forward;
                        Vector3 jointVector = m_JointsPositions[i + 1] - m_JointsPositions[i];
                        float angle = Vector3.Angle(jointForward, jointVector);
                        if (angle > 45)
                        {
                            m_JointsPositions[i + 1] = m_JointsPositions[i] + Quaternion.AngleAxis(angle - 45, Vector3.Cross(jointVector, jointForward)) * jointVector;
                        }

                        m_JointsRotations[i + 1] = m_JointsRotations[i] * Quaternion.FromToRotation(jointForward, m_JointsPositions[i + 1] - m_JointsPositions[i]);
                    }

                    // If the iteration could not find more optimal joints chain placement, abort the algorithm and do not change anything in joints transforms.
                    float lastTargetDistance = targetDistance;
                    targetDistance = Vector3.Distance(m_JointsPositions[lastJointIndex], m_Target);

                    if (lastTargetDistance <= targetDistance)
                    {
                        return;
                    }
                }

                if (iterationCount >= 30)
                {
                    Debug.LogWarning($"{nameof(JointsChain)}: FABRIK infinite iteration");  // TODO
                }
            }

            // Apply the new joints positions and rotations except for the origin joint which always remains at the same position.
            m_Joints[0].rotation = Quaternion.FromToRotation(Vector3.forward, m_JointsPositions[1] - m_JointsPositions[0]);     // TODO: optimize by using m_Joints[i].forward instead if Vector3.forward
            for (int i = 1; i < m_Joints.Length - 1; i++)
            {
                m_Joints[i].SetPositionAndRotation(m_JointsPositions[i], Quaternion.FromToRotation(Vector3.forward, m_JointsPositions[i + 1] - m_JointsPositions[i]));  // TODO: use localRotations array
            }
            m_Joints[m_Joints.Length - 1].position = m_JointsPositions[m_JointsPositions.Length - 1];

            // TODO: smooth the movement between the old and the new joints positions if the joints are far away from the new position
        }

        #endregion FABRIK methods
    }
}