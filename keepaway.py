#!/usr/bin/env python
from subprocess import Popen

class Any:
    """
    Just an easy go-to class for arbitrary data.
    """

    def __init__(self, **args):
        if args:
            self.__dict__.update(args)


def launch_player(player_type, options):
    """Launcher for both keepers and takers."""
    from itertools import chain

    print "**** PLAYER POLICY ****  " + str(getattr(options, player_type + '_policy'))

    # Build up the options for the player process.
    # TODO $klog_opts $kdraw_opts $kweight_opts
    player_options = dict(
        e = int(getattr(options, player_type + '_learn')),
        b = int(options.getopen_learn),
        j = options.taker_count,
        k = options.keeper_count,
        p = options.port,
        q = getattr(options, player_type + '_policy'),
        t = player_type + 's', # Pluralize for team name. TODO Really?
        x = options.stop_after,
        y = options.start_learning_after,
        z = int(options.getopen_hand))

    # Handle optional args.
    def put_optional(key, name):
        value = getattr(options, name, None)
        if value:
            player_options[key] = value
    # TODO Append player indices? Standard keepaway.sh does, and
    # TODO LinearSarsaAgent saves for each player.
    # TODO However, for my own input, I don't want independent files. Hrmm.
    put_optional('f', player_type + '_output')
    put_optional('w', player_type + '_input')

    # LSPI input/output
    if player_type == 'keeper':
        put_optional('g', 'getopen_input')
        put_optional('u', 'getopen_output')

    # Change the dict to a sorted list of args.
    player_options = player_options.items()
    player_options.sort()
    player_options = [
        ('-%s' % option[0], str(option[1])) for option in player_options]
    player_options = list(chain(*player_options))

    # Build keepaway_player command, and fork it off.
    # TODO Always assume keepaway_player is here?
    command = [relative('./player/keepaway_player')] + player_options
    #print command
    #print " ".join(command)
    proc = Popen(command)
    return proc

def launch_monitor(options):
    from subprocess import Popen
    # I can't find any way to zoom automatically on startup. Oh well.
    monitor_options = [('server-port', options.port)]
    monitor_options = [
        '--%s=%s' % option for option in monitor_options]
#command = [relative('../rcssmonitor_qt4/src/rcssmonitor')] + monitor_options
    command = ['rcssmonitor'] + monitor_options
    # print command
    # print " ".join(command)
    Popen(command)


def launch_server(options):
    """
    Launches the server.
    Returns its process id.
    """
    from socket import gethostname
    from subprocess import Popen
    from time import strftime

    # Some helpful vars.
    log_name = '%s-%s' % (strftime('%Y%m%d%H%M'), gethostname())

    # Build up the server arguments. Alphabetical order follows.
    server_options = []

    # Coach/trainer mode.
    server_options += [
        ('coach', int(options.coach)),
        ('coach_port', options.coach_port)]

    # Hardcoded settings for keepaway play.
    server_options += [('forbid_kick_off_offside', 0)]
    server_options += [('half_time', -1)]
    
    # Either keepaway or trainer mode. Field size.
    server_options += [('keepaway', int(not options.coach))]
    server_options += [('keepaway_start', options.game_start)]
    server_options += [
        ('keepaway_length', options.field_length),
        ('keepaway_width', options.field_width)]

    if options.log_keepaway:
        server_options += [
            ('keepaway_logging', 1),
            ('keepaway_log_dir', options.log_dir),
            ('keepaway_log_fixed', 1),
            ('keepaway_log_fixed_name', log_name)];

    if options.log_game:
        server_options += [
            # TODO Parameterize compression?
            ('game_log_compression', 0),
            ('game_log_dir', options.log_dir),
            ('game_log_fixed', 1),
            ('game_log_fixed_name', log_name),
            # TODO Parameterize the version number?
            ('game_log_version', 5)];
    else:
        server_options += [('game_logging', 0)];

    # Server and online coach ports.
    server_options += [
        ('olcoach_port', options.online_coach_port),
        ('port', options.port)];

    # Hardcoded stamina inc. This was hardcoded in keepaway.sh.
    # TODO What's the effect, and what's default?
    # TODO Any changes to other new defaults in rcssserver to retain benchmarks?
    server_options += [('stamina_inc_max', 3500)];

    # Synch mode. TODO What's default, and does synch offset matter when not
    # TODO in synch mode?
    server_options += [
        ('synch_mode', int(options.synch_mode)),
        # Synch offset 80 was hardcoded in keepaway.sh.
        ('synch_offset', 80)]

    if options.log_text:
        server_options += [
            # TODO Parameterize compression?
            ('text_log_compression', 0),
            ('text_log_dir', options.log_dir),
            ('text_log_fixed', 1),
            ('text_log_fixed_name', log_name)];
    else:
        server_options += [('text_logging', 0)];

    # More hardcoded settings for keepaway play.
    server_options += [('use_offside', 0)]
#SANMIT: default should be 90 or 180? I guess I'll leave full vision for now. 
    # Vision limits. TODO What's the normal default?
    if not options.restricted_vision:
        server_options += [('visible_angle', 360)]
    # clever! hehe
    server_options = [
        'server::%s=%s' % option for option in server_options]

    # Build rcssserver command, and fork it off.
    # TODO Locate rcssserver executable reliably.
#command = [relative('../rcssserver/src/rcssserver')] + server_options
    command = ['rcssserver'] + server_options
    print "COMMAND: " + str(command)
    # print " ".join(command)
    popen = Popen(command)

    # Wait until the server is ready.
    wait_for_server(options.port)
    return popen.pid


def main():
    """
    Just parses shell options and kicks things off.
    """
    options = parse_options()
    return run(options)


def parse_options(args = None, **defaults):
    """
    Parses the given list of args, defaulting to sys.argv[1:].
    Provide [] for default options.
    """
    from optparse import OptionParser
    parser = OptionParser()
    default_port = defaults.get('port', 5800)
    default_size = defaults.get('size', 20)
    # TODO Apply other defaults below?
    parser.add_option(
        '--coach', action = 'store_true', default = False,
        help = "Use trainer instead of server referee.")
    parser.add_option(
        # TODO More options are needed before coach/trainer is ready.
        '--coach-port', type = 'int', default = None,
        help = "Offline trainer port.")
    parser.add_option(
        '--field-length', type = 'int', default = default_size,
        help = "Playing field x-axis size.")
    parser.add_option(
        '--field-width', type = 'int', default = default_size,
        help = "Playing field y-axis size.")
    parser.add_option(
        '--game-start', type = 'int', default = 8,
        help =
            "Game start delay time, although we also now kick off 1 second "
            "after takers appear.")
    parser.add_option(
        '--keeper-count', type = 'int', default = 3,
        help = "Number of keepers.")
    parser.add_option(
        '--keeper-input',
        help = "Input (file) name for keeper policy agent.")
    parser.add_option(
        '--keeper-learn', action = 'store_true', default = False,
        help = "Turn learning on for keepers.")
    parser.add_option(
        '--keeper-output',
        help = "Output (file) name for keeper policy agent.")
    parser.add_option(
        '--keeper-policy',
        # Allow --keeper-policy=ext=./whatever.so, so remove choices.
        # TODO Nicer syntax for extensions?
        #type = 'choice', choices = ['hand', 'hold', 'learned', 'rand'],
        default = 'rand',
        help = "The policy for the keepers to follow.")
    parser.add_option(
        '--log-dir', default = "./logs",
        help = "Directory for storing log files.")
    parser.add_option(
        '--log-game', action = 'store_true', default = False,
        help = "Save rcg log file.")
    parser.add_option(
        '--log-text', action = 'store_true', default = False,
        help = "Save rcl (message/command) log file.")
    parser.add_option(
        '--monitor', action = 'store_true', default = False,
        help = "Launch the monitor to watch the play.")
    parser.add_option(
        '--no-log-keepaway', action = 'store_false', default = True,
        dest = 'log_keepaway',
        help = "Do not save kwy log file.")
    parser.add_option(
        '--online-coach-port', type = 'int', default = None,
        help = "Online coach port.")
    parser.add_option(
        '--port', type = 'int', default = default_port,
        help = "RCSS server port.")
    parser.add_option(
        '--restricted-vision', action = 'store_true', default = False,
        help = "Restrict player vision to less than 360 degrees.")
    # TODO This isn't used at all in the keepaway program.
    parser.add_option(
        '--start-learning-after', type = 'int', default = -1,
        help = "Start learning after the given number of episodes.")
    # TODO This isn't used at all in the keepaway program.
    # TODO Perhaps kick off a monitor here to watch for episodes?
    parser.add_option(
        '--stop-after', type = 'int', default = -1,
        help = "Stop play after the given number of episodes.")
    parser.add_option(
        '--synch-mode', action = 'store_true', default = False,
        help = "Speed up with synchronous mode.")
    parser.add_option(
        '--taker-count', type = 'int', default = 2,
        help = "Number of takers.")
    parser.add_option(
        '--taker-input',
        help = "Input (file) name for taker policy agent.")
    parser.add_option(
        '--taker-learn', action = 'store_true', default = False,
        help = "Turn learning on for takers.")
    parser.add_option(
        '--taker-output',
        help = "Output (file) name for taker policy agent.")
    parser.add_option(
        '--taker-policy', default = 'hand',
        # Allow --keeper-policy=ext=./whatever.so, so remove choices.
        # TODO Nicer syntax for extensions?
        #type = 'choice', choices = ['hand', 'learned'],
        help = "The policy for the takers to follow.")
    
    # LSPI Options   
    parser.add_option('--getopen-learn', action = 'store_true', default = False, help = 'Turn on LSPI learning for getting open')   # Learning or not?
    parser.add_option('--getopen-input', help = 'Load weights file base name for LSPI agent')   # If no load weights is given, it will follow a random policy with w=0
    parser.add_option('--getopen-output', help = 'Save weights file base name for LSPI agent')  # Save weights here (a number will be appended based on keeper number)
   
    parser.add_option('--load-same', action = 'store_true', default = False, help = 'Load weights for all players from one file')
    parser.add_option('--single-learner', action = 'store_true', default = False, help = 'Use one learner and 2 handcoded agents')

    parser.add_option('--getopen-hand', action = 'store_true', default = False, help = 'Use the hand get-open policy')
    

    options = parser.parse_args(args)[0]
    # Set coach_port and online_coach_port here, if not set previously.
    # This will allow them to be based on the args-given port.
    # This way, things still work even if just given a new main port.
    if not options.coach_port:
        options.coach_port = options.port + 1
    if not options.online_coach_port:
        # Seems nicer to base this on coach port, in case it was given manually.
        options.online_coach_port = options.coach_port + 1
    return options


def relative(path):
    """
    Returns a full path, relative to the current script's dir.
    """
    from os.path import abspath, dirname, join
    full = abspath(join(dirname(__file__), path))
    return full


def run(options):
    """
    Run with an options object already given.
    Handy for calling from other scripts rather than a shell.
    """
    from time import sleep
    # print options
    # First, make sure a server isn't already running on this port.
    if server_running(options.port):
        raise RuntimeError(
            "Server already running on port {0}.".format(options.port))

    # Kick off server.
    server_pid = launch_server(options)

    # Then keepers.
    for i in xrange(options.keeper_count):
        
        # Launch each player with their own load/save weights file if one is specified, 
        # since all out methods learn a separate policy for each agent
        
        if not options.load_same:
            if options.getopen_input is not None:
                options.getopen_input = options.getopen_input[0:-1] + str(i+1)
            if options.keeper_input is not None:
                options.keeper_input = options.keeper_input[0:-1] + str(i+1)
        if options.getopen_output is not None:
            options.getopen_output = options.getopen_output[0:-1] + str(i+1)
        if options.keeper_output is not None:
            options.keeper_output = options.keeper_output[0:-1] + str(i+1)
        # Make players 2 and 3 use handcoded getopen
        if options.single_learner:
            if i > 0:
                options.getopen_hand = True 

        proc = launch_player('keeper', options)
    # Watch for the team to make sure keepers are team 0.
    wait_for_players(options.port, 'keepers')

    # Then takers.
    for i in xrange(options.taker_count):
        launch_player('taker', options)
    # Allow dispstart to kick off play.
    wait_for_players(options.port, 'takers', True)

    # Then monitor.
    if options.monitor:
        launch_monitor(options)

    # All done.
    proc.communicate();
    return Any(server_pid = server_pid)


def server_running(port):
    """Impersonate a monitor to see if the server is running already."""
    from socket import AF_INET, SOCK_DGRAM, socket
    from time import sleep
    sock = socket(AF_INET, SOCK_DGRAM)
    try:
        sock.setblocking(False)
        sock.bind(('', 0))
        # Technically, this allows for race conditions, but it should cover
        # common cases of accidentally trying to kick off an already running
        # server.
        try:
            sock.sendto(
                '(dispinit version 4)', ('127.0.0.1', port))
            # TODO Sleep seems needed here, but I don't sleep here on the wait
            # TODO version.
            # TODO Does it only work because we catch the reply in the next
            # TODO round?
            sleep(0.1)
            # Sample rcssclient uses buffer size 8192.
            # TODO Do I care to validate these? data, sender =
            sock.recvfrom(8192)
            sock.sendto('(dispbye)', ('127.0.0.1', port))
            # It's running.
            return True
        except:
            # TODO Check to make sure it's the right kind of error?
            return False
    finally:
        sock.close()


def wait_for_players(port, team_name, go = False):
    """
    Impersonate a monitor to see if players are connected yet.
    TODO I have hacked multiple monitors here, and I ought to consider unifying
    TODO them.
    TODO This one has some corrected logic, such as updating the port.
    """
    from socket import AF_INET, SOCK_DGRAM, socket
    from time import sleep
    sock = socket(AF_INET, SOCK_DGRAM)
    try:
        sock.setblocking(False)
        sock.bind(('', 0))
        # Loop until successful.
        connected = False
        while True:
            # Delay a bit between attempts, so we don't bother the server nor
            # the user console.
            sleep(0.25)
            try:
                if not connected:
                    sock.sendto(
                        '(dispinit version 4)', ('127.0.0.1', port))
                # Sample rcssclient uses buffer size 8192.
# SANMIT: Not sure what the 127.0.0.1 is for... 
                while True:
                    data, sender = sock.recvfrom(8192)
                    connected = True
                    # Update the port for further communications.
                    port = sender[1]
                    if team_name in data:
                        break
                if go:
                    # Delay just a bit. If we start immediately after we see
                    # the takers, they don't seem to behave sanely, at least
                    # without synch-mode.
                    # I'm not sure, but I think it might relate to logic in
                    # SenseHandler::synchronize.
                    # I spent some time looking at 'show' status updates to see
                    # if I could find a sure pattern to know when we're safe.
                    # Might be something there, but I'm not sure how general
                    # anything is.
                    sleep(1.0)
                    sock.sendto('(dispstart)', ('127.0.0.1', port))
                    sock.recvfrom(8192)
                    print("Sent dispstart!")
                sock.sendto('(dispbye)', ('127.0.0.1', port))
                # Good to go.
                break
            except Exception as e:
                # TODO Check to make sure it's the right kind of error?
                pass
    finally:
        sock.close()


def wait_for_server(port):
    """Impersonate a monitor to see if the server is running yet."""
    from socket import AF_INET, SOCK_DGRAM, socket
    from time import sleep
    sock = socket(AF_INET, SOCK_DGRAM)
    try:
        sock.setblocking(False)
        sock.bind(('', 0))
        # Loop until successful.
        while True:
            # Delay a bit between attempts, so we don't bother the server nor
            # the user console.
            sleep(0.25)
            try:
                sock.sendto(
                    '(dispinit version 4)', ('127.0.0.1', port))
                # Sample rcssclient uses buffer size 8192.
                # TODO Do I care to validate these? data, sender =
                sock.recvfrom(8192)
                sock.sendto('(dispbye)', ('127.0.0.1', port))
                # Good to go.
                break
            except:
                # TODO Check to make sure it's the right kind of error?
                pass
    finally:
        sock.close()


if __name__ == '__main__':
    main()
