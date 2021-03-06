using UnityEngine;

namespace CrispyPhysics
{
    public interface IMomentum
    {
    	uint tick { get; }
        Vector2 force { get; }
        float torque { get; }
        Vector2 linearVelocity { get; }
        float angularVelocity { get; }
        Vector2 position { get; }
        float angle { get; }
        bool enduringContact { get; }
    }
}