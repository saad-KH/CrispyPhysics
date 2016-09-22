namespace CrispyPhysics
{
    public interface IContact
    {
        IBody firstBody { get; }
        IBody secondBody { get; }
        float friction { get;}
        float restitution { get; }
        float tangentSpeed { get; }
    }
}