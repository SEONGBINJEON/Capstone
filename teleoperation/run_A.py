"""Compatibility launcher for A, using the persistent teleoperation service."""
import sys
from run_teleop import main

if __name__=='__main__':
    if '--pair' not in sys.argv:sys.argv.extend(['--pair','A'])
    main()
