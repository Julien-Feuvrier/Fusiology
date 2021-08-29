using UnityEngine;

namespace Fusiology.Tools
{
    /// <summary>
    /// Component which applies a constant translation movement to the attached <see cref="Transform"/>.
    /// </summary>
    public class ConstantMove : MonoBehaviour
    {
        /// <summary>
        /// The translation vector which corresponds to the movement per second of the attached <see cref="Transform"/>.
        /// </summary>
        [SerializeField]
        private Vector3 m_MoveVector = Vector3.forward;

        /// <summary>
        /// Translate the transform each frame.
        /// </summary>
        private void Update() => transform.localPosition += m_MoveVector * Time.deltaTime;
    }
}