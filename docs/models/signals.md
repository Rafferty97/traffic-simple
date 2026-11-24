# Signals

How traffic lights change, behind the module boundary `models::signal`, so actuated control can come later. Compilation derives the plans from how an intersection is controlled (see [the design](../DESIGN.md), Intersection control), and signal barriers read the lights (see [Gap acceptance](gap-acceptance.md)).

They run fixed-time plans. A controller is part of the network, and its state, the phase it is in and how long each movement has been in its state, is part of the saved state. The movements' lights are one flat array in the state, every controller's together, each controller writing its own; a signal barrier holds its movement's index into it, so reading its light is one lookup and nothing is copied.

- **Movements:** each is red, amber or green, with its amber and clearance times and a bitmask of the movements it conflicts with.
- **Phases:** each is a mask of movements to be green, with a duration counted from when all of them are green, so clearance doesn't eat into green time.
- **Changes:** a movement not green in the current phase turns amber, then red after its amber time; one green in it turns green once every conflicting movement has been red for that movement's clearance time. When the phase's duration has passed, the next phase begins.
- **Update:** each step computes every movement's next state from the current states, then applies them all. Times are counted in steps, so they never drift.

This is how the old prototype ran its signals.
