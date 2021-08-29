using UnityEngine;

/// <summary>
/// Custom extensions class for the <see cref="UnityEngine.Vector3"/> structure.
/// </summary>
public static class Vector3Extensions
{
    /// <summary>
    /// Change the X component of this vector.
    /// </summary>
    /// <param name="vector">This vector.</param>
    /// <param name="value">The new X component value.</param>
    /// <returns>The updated vector.</returns>
    public static Vector3 SetX(this Vector3 vector, float value)
    {
        vector.x = value;
        return vector;
    }

    /// <summary>
    /// Change the Y component of this vector.
    /// </summary>
    /// <param name="vector">This vector.</param>
    /// <param name="value">The new Y component value.</param>
    /// <returns>The updated vector.</returns>
    public static Vector3 SetY(this Vector3 vector, float value)
    {
        vector.y = value;
        return vector;
    }

    /// <summary>
    /// Change the Z component of this vector.
    /// </summary>
    /// <param name="vector">This vector.</param>
    /// <param name="value">The new Z component value.</param>
    /// <returns>The updated vector.</returns>
    public static Vector3 SetZ(this Vector3 vector, float value)
    {
        vector.z = value;
        return vector;
    }

    /// <summary>
    /// Oppose the X component of this vector.
    /// </summary>
    /// <param name="vector">This vector.</param>
    /// <returns>A new vector whose X component is the opposite of the <paramref name="vector"/> one.</returns>
    public static Vector3 OpposeX(this Vector3 vector)
    {
        vector.x = -vector.x;
        return vector;
    }

    /// <summary>
    /// Oppose the Y component of this vector.
    /// </summary>
    /// <param name="vector">This vector.</param>
    /// <returns>A new vector whose Y component is the opposite of the <paramref name="vector"/> one.</returns>
    public static Vector3 OpposeY(this Vector3 vector)
    {
        vector.y = -vector.y;
        return vector;
    }

    /// <summary>
    /// Oppose the Z component of this vector.
    /// </summary>
    /// <param name="vector">This vector.</param>
    /// <returns>A new vector whose Z component is the opposite of the <paramref name="vector"/> one.</returns>
    public static Vector3 OpposeZ(this Vector3 vector)
    {
        vector.z = -vector.z;
        return vector;
    }
}