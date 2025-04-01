'''
code will run the adjust_ats function every x seconds
it will print the new ats position then simulate physics with the new surface area
'''


import time
import math

# Constants  
GRAVITY = 32.174   #ft/s^2
ROCKET_MASS = 17.8125 #mass of full scale
ROCKET_CROSS_SECTIONAL_AREA = 0.08814130888
MAX_FLAP_SURFACE_AREA = 0.02930555555
ATMOSPHERE_FLUID_DENSITY = 0.076474 # lbs/ft^3
ROCKET_DRAG_COEFFICIENT = 0.46
ATS_MAX_SURFACE_AREA = 0.02930555555

# Global variables (initialize these values appropriately)
gLaunchTime = time.time()   
gVelocityFiltered = 10.0  # Example value, replace with actual
gAltFiltered = 1000.0  # Example value, replace with actual
absolute_alt_target = 1500.0  # Example target altitude
gAccelFiltered = 5.0  # Example value, replace with actual
gAtsPosition = 0  # Initial ATS position
gAtsPosition = 0
old_error = 0
def PIDFactor(error, Kp, Kd):
    global old_error
    # Proportional term
    proportional = error * Kp
    # Derivative term
    derivative = (error - old_error[0]) * Kd
    # Update old_error for the next call
    old_error = error
    # Return the sum of proportional and derivative terms
    return proportional + derivative

def setATSPosition(position):
    # Example function to set the ATS position (replace with actual implementation)
    print(f"Setting ATS position to: {position}")

def adjust_ats():
    global gAtsPosition
    #! WRONG! need to fix it
    targetAcceleration = abs(gVelocityFiltered ** 2 / (2 * (absolute_alt_target - gAltFiltered)))
    
    # Retract ATS fully after 18 seconds
    if time.time() - gLaunchTime > 18:
        setATSPosition(0)
        return
    
    # Fully deploy ATS if reached Altitude target
    if gAltFiltered >= absolute_alt_target:
        gAtsPosition = 1
    else:
        # Calculate desired surface-area to reach target altitude
        target_area = (gVelocityFiltered ** 2 / (absolute_alt_target - gAltFiltered) - 2 * GRAVITY) * ROCKET_MASS / (gVelocityFiltered * ATMOSPHERE_FLUID_DENSITY * ROCKET_DRAG_COEFFICIENT)
        print(f"Old ATS pos: {target_area / ATS_MAX_SURFACE_AREA}")
        
        # Calculate error in acceleration
        error = gAccelFiltered - targetAcceleration  # positive if drag + gravity >= target
        # Calculate adjustment
        if error > 0:  # Too slow
            adjustment = 0
        else:  # Too fast
            adjustment = PIDFactor(abs(error), 0.03, 0)  # Normalize to 0 to 1
        
        gAtsPosition = adjustment
    
    # ATS window (adjust after 4.5 seconds)
    if time.time() - gLaunchTime > 4.5:
        # Adjust ATS based on position
        setATSPosition(gAtsPosition)
        print(f"ATS position: {gAtsPosition}")

def physics_from_ats():
    Fd = 1/2 * ROCKET_DRAG_COEFFICIENT * (ROCKET_CROSS_SECTIONAL_AREA + MAX_FLAP_SURFACE_AREA*
                                          gAtsPosition) * pow(gVelocityFiltered,2.0)
    inst_acceleration = Fd/ROCKET_MASS

def simulate_physics():
    pass

def main():
    start = time.time()
    last_timestep = start
    while True:
        curr_time = time.time()
        if (curr_time-last_timestep>1):
            last_timestep=curr_time
            adjust_ats()

main()

