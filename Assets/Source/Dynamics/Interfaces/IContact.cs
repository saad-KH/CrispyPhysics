namespace CrispyPhysics
{
    public interface IContact
    {
        IBody bodyA { get; }
        IBody bodyB { get; }
        float friction { get;}
        float restitution { get; }
        float tangentSpeed { get; }
    }
}