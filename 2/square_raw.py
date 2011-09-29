#!/usr/bin/env python

## Square Navigator
#
# This is the "raw" version of a navigation component to drive ERTS in a square pattern
#
# See local variable <tt>nav</tt> in main().
# The assignment is to tune <tt>nav['steering_sensitivity']</tt> to eliminate
# oversteer/understeer.
#
# Feel free to experiment with other parameters in <tt>nav</tt> as well, but
# do it one variable at a time
# 
import os
import select
import sys
import termios
import tty

import cjson

#
# "Connectivity" mappings to CartFS
#

## ERTS mount point
ROOT =    '/tmp/cartfs'
## synchronization point
CLOCK =   'clock'
## vehicle control interface
VCS =     'vcs/vcs_s'
## compass sensor
COMPASS = 'compass/compass_s'
## GPS sensor
GPS = 'gps/gps_s'
## driver interface
DRIVER = 'jdriver/jdriver_s'

## Switch to a raw TTY
#
# Switch terminal mode to "raw" to prevent interuption with CNTL-C (see cartfs.py)
#
def block_ctrl_c():

    block_ctrl_c.old_term_settings = termios.tcgetattr(sys.stdin.fileno())
    tty.setraw(sys.stdin.fileno())

## Switch out of a raw TTY
#
# Reverses the effect of block_ctrl_c(), enabling CTRL-C interruption 
#
def unblock_ctrl_c():

    ctrl_c_found = False
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        capchars = os.read(0, 100)
         # sweep the characters and check to see if one is ctrl-c
        for c in capchars:
            # when we find the crtl-c, set an exit flag to use once
            # we restore the terminal settings
            if c == '\x03':
                ctrl_c_found = True
                break
    termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN,
                      block_ctrl_c.old_term_settings)
    if ctrl_c_found:
        raise KeyboardInterrupt

##Calculate a steering correction
#
# @param[in]  compass  A dictionary
# @param[in]  driver   Provides value of <var>direction</var>
#
# \return t The steering correction in the form of an inverse turning radius
#
# \return diff The current heading error, the difference between current and actual headings.  A negative
# value indicates that the target is to the left of the current heading; a positive value indicates that the
# target is to the right.
#
# \pre
# - <var>compass['heading'] The actual heading.
# - <var>driver['direction'] The target heading.
#
# Compare the current and target headings to compute a new steering command.  The steering control unit
# is the inverse of the turning radius (ITR).  For example, to steer in a 3-meter # radius, the steering
# command is ITR = 0.3333...
#
# The steering correction is a new steering radius (as opposed to an increment).
# Its value is proportional to the error, with <var>driver['turn_P_term']</var>
# holding the proportionality factor.
#
# Steering is clipped to a radius held in <var>driver['corner_radius']</var>
#

def calc_inv_turn(compass, nav):
    '''Calculate the (signed) difference between the current heading and
    the target heading. Negative values indicate the target is to the left
    of the current heading. Maximum difference is 180 degrees. The
    difference is returned along with a turning control value which is the
    inverse of a circle radius for the turn.'''

    # Constrain heading to (-180, 180)
    if compass['heading'] > 180:
        compass['heading'] -= 360.0

    # Normalize target heading to (-180, 180)
    if nav['target_heading'] > 180:
        nav['target_heading'] -= 360

    # Find the error between our heading and our desired heading
    heading_error = float(compass['heading']) - float(nav['target_heading'])

    # Normalize heading error to (-180, 180)
    if heading_error > 180.0:
        heading_error -= 360.0
    elif heading_error < -180.0:
        heading_error += 360.0

    # Calculate the steering correction
    steering_correction = float(nav['steering_sensitivity']) * heading_error

    # Get the smallest turn allowed
    max_turn = 1 / float(nav['sq_corner'])

    # Clip the turn command to (-max_turn, max_turn)
    if steering_correction > max_turn:
        steering_correction = max_turn
    elif steering_correction < -max_turn:
        steering_correction = -max_turn

    return steering_correction, heading_error

#wangho
import geopy
from geopy import distance
 
def geocode_distance((x1, y1), (x2, y2), unit='km'): 
    if (x1, y1) == (x2, y2): 
        return 0 
    return distance.distance((x1, y1), (x2, y2)).kilometers 

from gps_points_bearing import bearing

#initialize nav parameters & nav_course['next_turn_diff']
def init_next_hop(start_latlon,end_latlon,nav,nav_course,course):              
    nav['target_heading'],meters_to_target = bearing(start_latlon,end_latlon), \
    geocode_distance(start_latlon,end_latlon) * 1000    
    
    next_next_point = (nav_course['next_point'] + 1) % len(course)
    bearing_diff = bearing(end_latlon,(course[next_next_point][1],course[next_next_point][2])) - float(nav['target_heading'])    

    if (bearing_diff >= 0):
	nav_course['next_turn_diff'] = 1
    else:
	nav_course['next_turn_diff'] = -1

    delta_distance = meters_to_target - float(nav['sq_corner'])
    if delta_distance <= 0:
	nav['control'] = 'turn'
    else:
	nav['control'],nav['sq_side'] = 'straight',delta_distance


#wangho

## Square Driver task
#
# @param[in]     compass  A dictionary representing the status of the compass
# @param[in]     vcs      A dictionary representing the status of the Vehicle Control Module
# @param[in,out] nav      A dictionary representing the driver state
#
#\return turn The desired inverse turning radius command
#\pre
# - <var>compass['heading']</var> is vehicle's current true heading in degrees relative to magnetic north
# - <var>vcs['distance'] is the cumulative distance travelled since start-up, in meters
# - <var>driver['direction']</var> is the desired direction of travel (North, South, East, West).
# - <var>driver['turn_state']</var> is the driver's control state (<tt>turn</tt> or <tt>straight</tt>)
# - <var>driver['steering_slop']</var> compass point at which control moves from <tt>turn</tt> to <tt>straight</tt>
#
#\post
# - <var>nav['control']</var> is the control state of the square-driver,
#   <tt>turn</tt> or <tt>straight</tt>
# - <var>nav['mark']</var> marks where vehicle left the last turn.  If the vehicle is just
#   now leaving a turn, this is reset to the value <var>vsc['distance']</var>

#wangho add parameters nav_course, course
def get_next_turn_radius_inverse(vcs, compass, nav, nav_course, course):
    '''
    Controller for driving in a rounded square, sides of length
    2*corner_radius_inv+side_length. Each call gives an inverse radius
    steering value.
    '''

    # Calcuate the next steering command
    steering_correction, heading_error = calc_inv_turn(compass, nav)

    # Check which part of the square we are on
    if nav['control'] == 'turn':
        # Check whether we have finished the corner
        if abs(heading_error) <= float(nav['steering_slop']): 
            nav['control'] = 'straight'
            nav['mark'] = float(vcs['distance']) + float(nav['sq_side'])
            #wangho
            if False == nav_course['next_init_flag']:
            	nav_course['next_init_flag'],nav_course['next_point'] = True,(nav_course['next_point'] + 1) % len(course)
            	print nav_course['next_init_flag'], nav_course['next_point']
            #wangho
    elif nav['control'] == 'straight':
        # Check whether we have finished the side
        delta_distance = float(nav['mark']) - float(vcs['distance'])
        if delta_distance <= 0:
            nav['control'] = 'turn'
            nav['target_heading'] = (float(nav['target_heading']) + 90 * float(nav_course['next_turn_diff'])) % 360
 
    # Return steering command and heading error
    return steering_correction, heading_error

## The component cycle
#
#

def main():
      
    # Change our working directory to the filesystem
    os.chdir(ROOT)

    # Open up the reads
    clock_fd = os.open(CLOCK, os.O_RDONLY)
    vcs_fd = os.open(VCS, os.O_RDONLY)
    compass_fd = os.open(COMPASS, os.O_RDONLY)
    gps_fd = os.open(GPS, os.O_RDONLY)

    # Ensure the paths exist for the writes
    path = os.path.split(DRIVER)[0]
    if path != '' and not os.path.exists(path):
        os.makedirs(path)
        os.chmod(path, 0777)

    # Open up the writes
    driver_fd = os.open(DRIVER, os.O_WRONLY + os.O_CREAT, 0666)

    
    ## Navigation parameters
    nav = {
          'mode': 'auto',                # autonomous mode
          'enable': True,                # Driver enabled
          'sq_side': 10.0,               # meters
          'sq_corner': 1.5,              # radius in meters #wangho 3.0->1.5
          'speed': 50.0,                 # fixed for this assignment
          'steering_slop': 5.0,          # degrees
                                         ###################
          'steering_sensitivity': 0.003, # TUNE THIS VALUE #
                                         ###################
          'control': 'turn',             # {'turn', 'straight'}
          'target_heading': 0.0,         # degrees from magnetic north
          'mark': 0.0,                   # starting position in meters
           }

    tick = 0

    #wangho
    ## Course parameters
    course = [[1,39.181917,    -86.5221208333,1.5,3.0],\
              [2,39.1818975,   -86.521724,    1.5,3.0],\
              [3,39.182143,    -86.5217033333,1.5,3.0],\
    #         [4,39.182199,    -86.5220985,   1.5,3.0],\
    #         [5,39.1819156667,-86.522309,    1.5,3.0],\
    #         [6,39.1819645,   -86.522398,    1.5,3.0],\
    #         [7,39.1820415,   -86.5223095,   1.5,3.0],\
    #         [8,39.1821313333,-86.5223926667,1.5,3.0],\
    #         [9,39.1822116667,-86.522302,    1.5,3.0] \
             ]

    #initialize
    nav_course = {
    	'next_point': 0, 
	'next_init_flag': True,
	'next_turn_diff': 1,
	}
    #wangho

    while True:
        # Wait for the clock
        block_ctrl_c()
        clock_stat = os.fstat(clock_fd)
        unblock_ctrl_c()

        # Read the clock
        os.lseek(clock_fd, 0, os.SEEK_SET)
        clock_json = os.read(clock_fd, clock_stat.st_size)
        clock = cjson.decode(clock_json)

        # Read the VCS
        os.lseek(vcs_fd, 0, os.SEEK_SET)
        vcs_json = os.read(vcs_fd, os.fstat(vcs_fd).st_size)
        vcs = cjson.decode(vcs_json)

        # Read the compass
        os.lseek(compass_fd, 0, os.SEEK_SET)
        compass_json = os.read(compass_fd, os.fstat(compass_fd).st_size)
        compass = cjson.decode(compass_json)

        # Read the GPS
        os.lseek(gps_fd, 0, os.SEEK_SET)
        gps_json = os.read(gps_fd, os.fstat(gps_fd).st_size)
        gps = cjson.decode(gps_json)

        #wangho
        if True == nav_course['next_init_flag']:
            #read from GPS and course list
            current_latlon,waypoint_latlon = (float(gps['lat']),float(gps['lon'])),\
		(course[nav_course['next_point']][1],course[nav_course['next_point']][2])
	    #initialize nav parameters & nav_course['next_turn_diff']
	    init_next_hop(current_latlon,waypoint_latlon,nav,nav_course,course)
	    #set flag
	    nav_course['next_init_flag'] = False

        print nav_course['next_point'], nav_course['next_init_flag'], nav['control'], nav['sq_side'], nav['target_heading']
        #wangho

        # Get the steering correction and error
        turn_cmd, heading_error = get_next_turn_radius_inverse(vcs, compass, nav, nav_course, course)
        print nav_course['next_init_flag'], nav_course['next_point']
        # Set up command to the vehicle.
        driver = {}
        driver['clock'] = clock['clock']   # use the clock's tick, no sync check
        driver['mode'] = 'auto'            # autonomous mode
        driver['enable'] = True            # this driver enabled
        driver['direction'] = 'forward'    # {'forward', 'backward'}
        driver['percent_throttle'] = nav['speed']  # set to keep speed relatively low
        driver['percent_braking'] = 0.0            # brakes off
        driver['turn_radius_inverse'] = turn_cmd   # new steering command

        tick = (tick + 1) % 10
        if tick == 0:
           print nav['control'], heading_error

        # Write the driver status
        driver_json = cjson.encode(driver)
        os.ftruncate(driver_fd, len(driver_json))
        os.lseek(driver_fd, 0, os.SEEK_SET)
        os.write(driver_fd, driver_json)

if __name__ == '__main__':
    main()
