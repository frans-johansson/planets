# Planets
Another one of those "let's write this in a weekend", kind of things. All the code is currently just in a single C file. At some point I might clean it up, but for now, it works.

The project depends on Raylib. Make sure you have it installed and accessible for linking if you want to try and build this yourself.

## Controls
- `W, A, S, D`: Pan the camera
- `HARD BRACKETS`: Zoom in and out
- `UP, DOWN`: Change the simulation speed
- `SPACE`: Pause and unpause the simulation
- `BACKSPACE`: Delete all celestial bodies, and start from scratch
- `RETURN`: Reset to a default two-body system
- `LEFT CLICK`: While not in "spawn mode", toggles information on the selected celestial body
- `Q`: Toggle "spawn mode", allowing you to add new celestial bodies with the mouse. Control initial velocity by dragging the mouse while holding down the left mouse button.
- `LEFT, RIGHT`: While in "spawn mode", controls the mass of the new celestial body
- `CTRL+LEFT, CTRLi+RIGHT`: While in "spawn mode", controls the radius of the new celestial body
- `PERIOD, COMMA`: While in "spawn mode", rotates between colors for the new celestial body
