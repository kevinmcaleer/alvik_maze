# Maze Explorer - Blockly Visual Representation

This document shows what the Maze Explorer code would look like as Blockly blocks.

## Main Program Structure

```
┌─────────────────────────────────────────────┐
│ 🟦 WHEN PROGRAM STARTS                      │
└─────────────────────────────────────────────┘
           │
           ▼
┌─────────────────────────────────────────────┐
│ 🟩 SET [wall_distance] TO [15]              │
└─────────────────────────────────────────────┘
           │
           ▼
┌─────────────────────────────────────────────┐
│ 🟩 SET [move_step] TO [5]                   │
└─────────────────────────────────────────────┘
           │
           ▼
┌─────────────────────────────────────────────┐
│ 🟩 SET [turn_angle] TO [90]                 │
└─────────────────────────────────────────────┘
           │
           ▼
┌─────────────────────────────────────────────┐
│ 🟨 PRINT "Maze Explorer Starting!"          │
└─────────────────────────────────────────────┘
           │
           ▼
┌─────────────────────────────────────────────┐
│ 🔁 REPEAT FOREVER                           │
│    ┌─────────────────────────────────────┐  │
│    │ 🟩 SET [distance] TO                │  │
│    │    🔵 GET DISTANCE FROM SENSOR      │  │
│    └─────────────────────────────────────┘  │
│           │                                  │
│           ▼                                  │
│    ┌─────────────────────────────────────┐  │
│    │ 🟨 PRINT "Distance: " [distance]    │  │
│    └─────────────────────────────────────┘  │
│           │                                  │
│           ▼                                  │
│    ┌─────────────────────────────────────┐  │
│    │ 🔷 IF [distance] < [wall_distance]  │  │
│    │    THEN:                            │  │
│    │    ┌──────────────────────────────┐ │  │
│    │    │ 🟨 PRINT "WALL DETECTED!"    │ │  │
│    │    └──────────────────────────────┘ │  │
│    │    ┌──────────────────────────────┐ │  │
│    │    │ 🔵 ROTATE LEFT [turn_angle]  │ │  │
│    │    └──────────────────────────────┘ │  │
│    │    ┌──────────────────────────────┐ │  │
│    │    │ ⏱️ WAIT 0.5 SECONDS          │ │  │
│    │    └──────────────────────────────┘ │  │
│    │    ELSE:                            │  │
│    │    ┌──────────────────────────────┐ │  │
│    │    │ 🟨 PRINT "Path clear"        │ │  │
│    │    └──────────────────────────────┘ │  │
│    │    ┌──────────────────────────────┐ │  │
│    │    │ 🔵 MOVE FORWARD [move_step]  │ │  │
│    │    └──────────────────────────────┘ │  │
│    │    ┌──────────────────────────────┐ │  │
│    │    │ ⏱️ WAIT 0.3 SECONDS          │ │  │
│    │    └──────────────────────────────┘ │  │
│    └─────────────────────────────────────┘  │
└─────────────────────────────────────────────┘
```

## Block Categories Used

### 🟦 Events
- `WHEN PROGRAM STARTS` - Triggers when the program begins

### 🔁 Control/Loops
- `REPEAT FOREVER` - Infinite loop

### 🔷 Logic
- `IF...THEN...ELSE` - Conditional statement
- `< (less than)` - Comparison operator

### 🟩 Variables
- `SET [variable] TO [value]` - Create and assign variables
- `[variable]` - Use variable value

### 🔵 Robot Movement
- `GET DISTANCE FROM SENSOR` - Read ToF sensor
- `ROTATE LEFT [degrees]` - Turn robot left
- `MOVE FORWARD [cm]` - Drive robot forward

### 🟨 Output
- `PRINT [text]` - Display message

### ⏱️ Timing
- `WAIT [seconds] SECONDS` - Pause execution

## Simplified Flowchart

```
START
  ↓
Set wall_distance = 15 cm
  ↓
Set move_step = 5 cm
  ↓
Set turn_angle = 90°
  ↓
┌──────────────────────┐
│  Get distance        │◄──────┐
└──────────────────────┘       │
  ↓                            │
  Is distance < 15cm?          │
  ↓              ↓              │
 YES            NO              │
  ↓              ↓              │
Turn left      Move forward    │
90 degrees     5 cm             │
  ↓              ↓              │
Wait 0.5s      Wait 0.3s        │
  ↓              ↓              │
  └──────────────┴──────────────┘
        (loop forever)
```

## Key Algorithm Steps

1. **Initialize** - Set up distance threshold, movement step, and turn angle
2. **Sense** - Read distance from ToF sensor
3. **Decide** - Is there a wall ahead?
   - **Wall detected** → Turn left 90°
   - **Path clear** → Move forward 5cm
4. **Repeat** - Go back to step 2

## Educational Notes

This is a **left-hand rule** maze solver:
- Always turns LEFT when hitting a wall
- Will eventually explore all connected areas
- Simple but effective for basic mazes
- Can get stuck in loops in some maze designs

To make a **right-hand rule** instead:
- Change `turn_angle` to `-90` (negative = turn right)
