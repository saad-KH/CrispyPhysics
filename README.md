# Crispy Physics
Crispy Physics is a 2D Physics Engine written in C# for Unity3D.

It's a close reapropriation of [Box2D](https://github.com/erincatto/Box2D) assuming a similar structure along with heavily borrowed code. 

The main feature of the engine is the ability to step and rollback the simulated world freely along the timeline foreseeing and keeping track of future and past states. 

[**Demo**](https://saadkh.com/crate/cripsy-physics)

## Features
**History and Roll Backs**: The engine maitains a history of the simulation so that roll backs to previous ticks are possible. Past states of the entities are also inspectable from the present tick.

**Foreseeable Future**: The engine presimulates forseeable ticks of the worlds so that each entity can expose its future states.

**Crispy Behavior**: It is possible to have a body deflected from the simulated curve at a given tick to an authored state, for motived or unmotived reasons. The engine allows this behavior while smoothing the deviation so that it tries to appear physically believable.


## Usage

Copy the library folder **Librairies/Crispy Physics** inside an Unity Project.

Include the Crispy Physics namespace inside your code use the using directive
```csharp
using CrispyPhysics;
``` 

Create the physics World.

```csharp
IWorld world = WorldFactory.CreateWorld(
    0.01f, // float for the fixed time step in seconds e.g., 0.01f
    gravity // Vector2 for the Physics Gravity e.g., new Vector2(0f, -9.8f);
);
```

The Crispy Physics engine runs on a fixed time step, it serves as the minimal step for each tick.

Create new bodies inside the world

```csharp
IBody body = world.CreateBody(
    position, // Vector2 for the current position in units
    angle, // float for the current angle in degrees
    bodyType, // BodyType (Dynamic, Static, Kinematic) representing the nature of the body
    shape // The IShape of the body see ShapeFactory
)
```
Simulate and step the world
```csharp
world.Step(
    ticks, // uint for the number of ticks to advance by
    foreseeTicks, // uint for how many future ticks to foresee
    bufferTicks, // uint for how many ticks that have to be predicted by the end of this step
    keepTicks // uint for how many past ticks to remember
);
```
To rollback to a previous tick
```csharp
world.RollBack(
    ticks // uint for how many ticks to rollback by
);
```

An example scene is provided at **Examples/A Game of Balls**
