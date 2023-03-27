# Autonomous Commands

## Start Positions
  1. Wall - Last cone column against the wall side
  2. Center Wall - Cones center grid towards the wall side
  3. Center Load Station - Cones center grid towards the load station side
  4. LoadStation - Last cone column against the load station

## Preload selections
  1. Do nothing (all positions)
  2. Preload score cone high (all positions)
  
## Path Selections
  1. Do nothing - Don't follow a path (Done & Tested)
  2. Center Balance+Intake - Cross CS, intake, and balance (Done & Tested)
  3. Sides Mobility + Intake - Intake and stop (Needs updating for new bot)
  4. Sides 2 Score - Intake and high score cube (Done & Tested)
  5. Sides Intake + Balance (Needs updating for new bot)
  5. Center Mobility+Balance (Not Done)
  6. Center Balance Only (Not Done)

Concept:  Instead of a whole host of separate commands, have configurable start position separate from sequence and apply logic to make sure consistent


# Initial start position
  ## Set odometry based on selected start position

# Initial scoring -- sequential
  ## Set scoring target as appropriate
  ## Move to scoring position CG -- use withTimeout?
  ## Open Claw -- with timeout or wait following
  ## Move to intake backstop -- withTImeout?
  
# Move sequences
  ## None -- just an empty command 

  ## Center balance -- just a path to follow
  ## Move for mobility -- just a path to follow


