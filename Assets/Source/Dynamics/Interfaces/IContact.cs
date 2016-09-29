using System;

namespace CrispyPhysics
{
    #region Events Definition
    public delegate void IContactHandlerDelegate(IContact contact, EventArgs args);
    #endregion

    public interface IContact
    {
        IBody firstBody { get; }
        IBody secondBody { get; }
        float friction { get;}
        float restitution { get; }
    }
}