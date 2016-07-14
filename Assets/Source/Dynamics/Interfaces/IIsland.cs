using UnityEngine;

namespace CrispyPhysics
{
    public interface IIsland
    {

        void Add(IBody body);

        void Add(IContact contact);

        void Solve(TimeStep step, Vector2 gravity, bool allowSleep);

        void SolveTOI(TimeStep subStep, int toiIndexA, int toiIndexB);

        void Report();

        void Clear();

    }
}

