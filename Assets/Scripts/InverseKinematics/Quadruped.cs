using System.Collections;
using UnityEngine;

namespace Fusiology.InverseKinematics
{
    /// <summary>
    /// Script which manages the movements and behaviour of a quadruped being.
    /// </summary>
    public class Quadruped : MonoBehaviour  // TODO: move body position/rotation from the current legs ends positions
    {
        #region Private fields

        /// <summary>
        /// Reference to the left forward leg joints chain whose movement is controlled by this component.
        /// </summary>
        [SerializeField]
        private JointsChain m_LeftForwardLeg;

        /// <summary>
        /// Reference to the right forward leg joints chain whose movement is controlled by this component.
        /// </summary>
        [SerializeField]
        private JointsChain m_RightForwardLeg;

        /// <summary>
        /// Reference to the left back leg joints chain whose movement is controlled by this component.
        /// </summary>
        [SerializeField]
        private JointsChain m_LeftBackLeg;

        /// <summary>
        /// Reference to the right back leg joints chain whose movement is controlled by this component.
        /// </summary>
        [SerializeField]
        private JointsChain m_RightBackLeg;

        /// <summary>
        /// The local position from the quadruped center of the default left forward leg end position.
        /// The default positions of the other legs is computed from this <see cref="Vector3"/> by opposing the X and/or Z components.
        /// </summary>
        [SerializeField]
        private Vector3 m_DefaultEndLegPosition;    // TODO: create a class IKLeg which configures this field for each leg (and integrate leg distance trigger, its JointsChain, the movement coroutine and its animation curves) + auto correct the target position with a raycast

        /// <summary>
        /// When the distance between a leg end position and its next target is above this distance, the leg movement is initiated.
        /// </summary>
        [SerializeField]
        private float m_LegDistanceTrigger = 1f;

        /// <summary>
        /// The time scale curve to apply during the animation of the legs.
        /// Its abscissa is defined between 0s and the duration of the leg movement animation, and the ordinate must be between 0 and 1.
        /// The curve must go from the origin to the (animation duration, 1) point.
        /// </summary>
        [SerializeField]
        private AnimationCurve m_LegAnimationTimeScale = new AnimationCurve(new Keyframe(0f, 0f, 1f, 1f), new Keyframe(1f, 1f, 1f, 1f));

        /// <summary>
        /// The curve which defines the height of the legs during their animation.
        /// The abscissa is the unscaled time between 0 and 1, and the ordinate the legs height in meters.
        /// The curve must go from the origin to the (1, 0) point (the last point reaches the destination point and has a height equals to 0).
        /// </summary>
        [SerializeField]
        private AnimationCurve m_LegAnimationHeight = new AnimationCurve(new Keyframe(0f, 0f, 0f, 0f), new Keyframe(0.5f, 1f, 0f, 0f), new Keyframe(1f, 0f, 0f, 0f));

        /// <summary>
        /// The current coroutine which animates a leg.
        /// <c>null</c> if no leg is currently moving.
        /// </summary>
        private Coroutine m_LegMovementCoroutine;

        #endregion Private fields

        #region Life-cycle methods

        /// <summary>
        /// Initialize the position of the quadruped legs.
        /// </summary>
        private void Start()
        {
            m_LeftForwardLeg.Target = m_LeftForwardLeg.transform.TransformPoint(m_DefaultEndLegPosition);
            m_RightForwardLeg.Target = m_RightForwardLeg.transform.TransformPoint(m_DefaultEndLegPosition.OpposeX());
            m_LeftBackLeg.Target = m_LeftBackLeg.transform.TransformPoint(m_DefaultEndLegPosition.OpposeX());
            m_RightBackLeg.Target = m_RightBackLeg.transform.TransformPoint(m_DefaultEndLegPosition);
        }

        /// <summary>
        /// Check each frame if a leg should be moved.
        /// </summary>
        private void Update()
        {
            if (m_LegMovementCoroutine == null)
            {
                Vector3 destination;
                if (Vector3.Distance(m_LeftForwardLeg.Target, destination = m_LeftForwardLeg.transform.TransformPoint(m_DefaultEndLegPosition)) > m_LegDistanceTrigger)
                {
                    m_LegMovementCoroutine = StartCoroutine(LegMovementCoroutine(m_LeftForwardLeg, destination));
                }
                else if (Vector3.Distance(m_RightForwardLeg.Target, destination = m_RightForwardLeg.transform.TransformPoint(m_DefaultEndLegPosition.OpposeX())) > m_LegDistanceTrigger)
                {
                    m_LegMovementCoroutine = StartCoroutine(LegMovementCoroutine(m_RightForwardLeg, destination));
                }
                else if (Vector3.Distance(m_LeftBackLeg.Target, destination = m_LeftBackLeg.transform.TransformPoint(m_DefaultEndLegPosition.OpposeX())) > m_LegDistanceTrigger)
                {
                    m_LegMovementCoroutine = StartCoroutine(LegMovementCoroutine(m_LeftBackLeg, destination));
                }
                else if (Vector3.Distance(m_RightBackLeg.Target, destination = m_RightBackLeg.transform.TransformPoint(m_DefaultEndLegPosition)) > m_LegDistanceTrigger)
                {
                    m_LegMovementCoroutine = StartCoroutine(LegMovementCoroutine(m_RightBackLeg, destination));
                }
            }
        }

        #endregion Life-cycle methods

        #region Leg control methods

        /// <summary>
        /// Coroutine which take care of the animation of a leg when it is moved from its current location to the specified destination.
        /// </summary>
        /// <param name="leg">The leg which is animated.</param>
        /// <param name="destination">The destination position of the leg in world space coordinates.</param>
        private IEnumerator LegMovementCoroutine(JointsChain leg, Vector3 destination)   // TODO: add randomness to the destination
        {
            float timer = 0f;
            float animationDuration = m_LegAnimationTimeScale.keys[m_LegAnimationTimeScale.keys.Length - 1].time;
            Vector3 origin = leg.Target;
            Vector3 movementVector = destination - origin;

            while (timer < animationDuration)
            {
                float unscaledTime = m_LegAnimationTimeScale.Evaluate(timer);
                leg.Target = origin + movementVector * unscaledTime + m_LegAnimationHeight.Evaluate(unscaledTime) * transform.up;

                yield return null;
                timer += Time.deltaTime;
            }

            // Move the leg to its final destination.
            leg.Target = destination;
            m_LegMovementCoroutine = null;
        }

        #endregion Leg control methods
    }
}