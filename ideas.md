
# Softbody Physics Ideas

## Parameter Space

The softbody can have several parameters from both the damped springs that governs the shell and the ideal gas law that governs the inside.

Damped spring equation:
(d/dt)^2 x + 2zw(dx/dt) + w^2 x = 0

where

w = sqrt(k/m), "damping ratio"

z = c / (2sqrt(mk)), "undamped angular frequency"

k: spring constant, "stiffness"
c: damping coefficient
m: mass
x: displacement
t: time

Important regions:

z > 1: overdamped, results in exponential decay
z = 1: critically damped, fastest return to steady state without oscillation
z < 1: underdamped, the spring oscillates while amplitude approaches 0

## Mood Space

Moods can be embedded (happiness, hunger, grumpiness, playfulness, etc.) into a space that respects relationships between moods (e.g. happy and sad are opposite).

A transfer function that depends on current mood state and time can be constructed (e.g. d(grumpiness)/dt is proportional to hunger)

## Potential Mappings

A mapping from the mood space to the softbody parameter space to display moods (e.g. happiness -> lower damping)
