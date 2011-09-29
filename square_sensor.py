#!/usr/bin/env python

import logging
import optparse
import os
import traceback

from cartfs import CartFSLoggingHandler, Sensor

# Default values for options
class options:
    # Cartfs File System Options
    debug = False
    root = '/tmp/cartfs'
    clock = 'clock'
    command = './driver/driver_c'
    status = './driver/driver_s'
    doc = './driver/driver_d'
    log = './driver/driver_log'

    vcs = './vcs/vcs_s'
    compass = './compass/compass_s'
    gps = './gps/gps_s'
    jdriver = './jdriver/jdriver_s'

class SquareDriver(Sensor):

    driver_c = {
        'clock': 0,
        'enable': True,
    }

    driver_c_doc = {
        'clock': "The clock value on which this data was written.",
        'enable': "True/False - stops reads on jdriver device",
    }

    driver_s = {
        'clock': 0,
        'enable': True,
        'direction': 0,
        'side_length': 10, # meters
        'corner_radius': 3, # meters
        'turn_state': 'turn',
        'turn_slop': 5, # degrees
        'turn_P_term': 0.03, # <<<!!! TUNE ME, please.
    }

    driver_s_doc = {
        'clock': "The clock value on which this data was written.",
        'enable': "True/False - stops reads on driver device",
        'direction': "The target heading of the driver",
        'side_length': "How long a side of the square should be (meters)",
        'corner_radius': "How large of a radius the corners should be (meters)",
        'turn_state': "What state of the turn we are in (turn/straight)",
        'turn_slop': "How close to straight we must be to be going straight",
        'turn_P_term': "The P term of the PID controller for steering",
    }

    jdriver_s = {
        'clock': 0,
        'enable': True,
        'percent_throttle': 0.0,
        'percent_braking': 0.0,
        'turn_radius_inverse': 0.0,
        'direction': 'forward',
        'mode': 'manual'
    }

    def __init__(self, options):
        Sensor.__init__(self)

        self.add_reader(options.clock, 'clock')
        self.add_reader(options.vcs, 'vcs_s')
        self.add_reader(options.compass, 'compass_s')
        self.add_reader(options.gps, 'gps_s')

        self.add_reader(options.command, 'driver_c')
        self.add_writer(options.status, 'driver_s', create=True)
        self.add_writer(options.jdriver, 'jdriver_s', create=True)

        self.write_once(options.command, self.driver_c, create=True)
        self.write_once(options.command + '_d', self.driver_c_doc,
                        create=True, write_always=True)
        self.write_once(options.status + '_d', self.driver_s_doc,
                        create=True, write_always=True)

    def calc_inv_turn(self):
        '''Calculate the (signed) difference between the current heading and
        the target heading. Negative values indicate the target is to the left
        of the current heading. Maximum difference is 180 degrees. The
        difference is returned along with a turning control value which is the
        inverse of a circle radius for the turn. The minimum radius allowed
        is 3 meters.'''

        heading = self.compass_s['heading']
        direction = self.driver_s['direction']

        # Constrain heading to (-180, 180)
        if heading > 180:
            heading -= 360.0

        # Constrain direction to (-180, 180)
        if direction > 180:
            direction -= 360
            self.driver_s['direction'] = direction

        # Find the error between our heading and our desired heading
        diff = float(heading) - float(direction)

        # Constrain heading error to (-180, 180)
        if diff > 180.0:
            diff -= 360.0
        elif diff < -180.0:
            diff += 360.0

        # Calculate the PID controller
        t = self.driver_s['turn_P_term'] * diff

        # Get the smallest turn allowed
        max_turn = 1 / float(self.driver_s['corner_radius'])

        # Constrain our turn to (-max_turn, max_turn)
        if t > max_turn:
            t = max_turn
        elif t < -max_turn:
            t = -max_turn

        return t, diff

    def get_next_turn_radius_inverse(self):
        '''Controller for driving in a rounded square, sides of length
        2*corner_radius_inv+side_length. Each call gives an inverse radius
        steering value.'''

        # Bring some values into variables to make things nicer
        heading = self.compass_s['heading']
        distance = self.vcs_s['distance']
        turn_slop = self.driver_s['turn_slop']
        turn_state = self.driver_s['turn_state']

        # Calcuate the next steering input
        turn, diff = self.calc_inv_turn()

        # Check which part of the square we are on
        if turn_state == 'turn':
            # Check whether we have finished the corner
            if abs(diff) <= turn_slop:
                print 'straight'
                self.driver_s['turn_state'] = 'straight'
                self.driver_s['straight_started_at'] = distance
            pass
        elif turn_state == 'straight':
            # Check whether we have finished the side
            delta_distance = distance - self.driver_s['straight_started_at']
            if delta_distance >= self.driver_s['side_length']:
                print 'turn'
                self.driver_s['turn_state'] = 'turn'
                self.driver_s['direction'] = (self.driver_s['direction'] - 90) % 360

        # Return our steering input
        return turn

    def process(self):
        # Ensure there is a clock value in driver_s
        if 'clock' not in self.driver_s:
            self.driver_s['clock'] = self.clock['clock'] - 1

        # Verify how many clock ticks have passed
        ticks = self.clock['clock'] - self.driver_s['clock']
        if ticks > 1:
            logging.error('missed %d clock cycles' % (ticks - 1,))

        # If there are any exceptions, we'll just print it out and continue
        try:
            # Adjust the status values of the 'jdriver' sensor, which we are
            # simulating to make use of the simpler control values
            self.jdriver_s['mode'] = 'auto'           # autonomous mode
            self.jdriver_s['percent_throttle'] = 50.0 # set to keep speed relatively low
            self.jdriver_s['percent_braking'] = 0.0   # brakes off

            # Pass along whether we are enabled
            self.jdriver_s['enable'] = self.driver_c['enable']

            # Get next steering control
            self.jdriver_s['turn_radius_inverse'] = self.get_next_turn_radius_inverse()

            # Print some helpful information for debugging
            print self.clock['clock'], self.gps_s['lat'], self.gps_s['lon'], \
                 self.compass_s['heading'], self.vcs_s['distance'], self.vcs_s['speed']
        except Exception:
            # Print the traceback so we can still debug
            traceback.print_exc()

if __name__ == '__main__':
    # Use psyco to speed things up if it's available.
    try:
        import psyco
        psyco.full()
    except ImportError:
        pass

    # Setup the command-line options for this module
    parser = optparse.OptionParser()
    parser.add_option('-d', '--debug', action='store_true', dest='debug')
    parser.add_option('-r', '--root', dest='root', metavar='ROOT')
    parser.add_option('-c', '--clock', dest='clock', metavar='CLOCK')
    parser.add_option('-C', '--command', dest='command', metavar='COMMAND')
    parser.add_option('-s', '--status', dest='status', metavar='STATUS')
    parser.add_option('-l', '--log', dest='log', metavar='LOG')
    parser.set_defaults(**options.__dict__)
    (options, args) = parser.parse_args()

    # Change directory into the root of the filesystem
    os.chdir(options.root)

    # Setup the debugging message format
    formatter = logging.Formatter('%(asctime)s %(name)-12s %(levelname)-8s %(message)s')

    # Setup the logging handler for logging to the filesystem
    log = CartFSLoggingHandler(options.clock, options.log)
    log.setLevel(logging.ERROR)
    log.setFormatter(formatter)
    logging.getLogger('').addHandler(log)

    # If we're in debug mode, then log to the console too
    if options.debug:
        console = logging.StreamHandler()
        console.setLevel(logging.DEBUG)
        console.setFormatter(formatter)
        logging.getLogger('').addHandler(console)

    # Instaniate our driver and run it
    sensor = SquareDriver(options)
    sensor.run()
