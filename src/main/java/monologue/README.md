# oh-yes-0-fps's monologue fork
Taken from FRC 3173

A logging library that supports datalog and nt.

- A complete rewrite of 2024.
- Supports datalog only, nt only or optimized for any piece of logged data.
- doesnt box primitives.
- can imperitively log even if a node has multiple locations.
- 40% reduction in annotation based logging cpu time.
- A procedural struct generator that doesn't box and works on enums, records and POJOs.