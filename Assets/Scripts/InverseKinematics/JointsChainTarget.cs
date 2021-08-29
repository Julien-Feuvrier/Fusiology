using UnityEngine;

namespace Fusiology.InverseKinematics
{
    /// <summary>
    /// Represents a <see cref="UnityEngine.Transform"/> 3D joints chain target which controls the attached <see cref="JointsChain"/> target.
    /// </summary>
    public class JointsChainTarget : MonoBehaviour
    {
        /// <summary>
        /// The joints chain whose target is controlled by this component.
        /// </summary>
        [SerializeField]
        private JointsChain m_JointsChain;

        /// <summary>
        /// Updates the joints chain target <see cref="Vector3"/> each frame.
        /// </summary>
        private void Update() => m_JointsChain.Target = transform.position;
    }
}