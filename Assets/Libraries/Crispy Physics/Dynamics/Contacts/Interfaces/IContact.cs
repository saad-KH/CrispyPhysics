using System;

namespace CrispyPhysics
{
    #region Events Definition
    public delegate void IContactHandlerDelegate(IContact contact, IContactMomentum momentum);
    #endregion

    public interface IContact
    {
        IBody firstBody { get; }
        IBody secondBody { get; }
        float friction { get;}
        float restitution { get; }
    }
}