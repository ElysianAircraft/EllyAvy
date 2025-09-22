"""
This is a basic example of running a coupled aircraft design-mission optimization in Aviary using
the "level 1" API.

The properties of the aircraft are defined in a pre-existing ".csv" file - in this case it describes
a conventional single-aisle commercial transport. The mission is defined using a "phase_info" file,
which consists of a climb, cruise, and descent phase.

The aircraft sizing problem is ran by calling the `run_aviary` function, which takes in the path to
the aircraft model, the phase info, and some other optional settings. This performs a coupled
design-mission optimization.
"""

import aviary.api as av
from aviary.utils.doctape import check_value

check_value(av.LegacyCode.FLOPS.value, 'FLOPS')
check_value(av.EquationsOfMotion.HEIGHT_ENERGY.value, 'height_energy')


phase_info = {
    'pre_mission': {'include_takeoff': False, 'optimize_mass': True},
    'climb_1': {
        'subsystem_options': {'core_aerodynamics': {'method': 'computed'}},
        'user_options': {
            'num_segments': 2,
            'order': 3,
            'mach_optimize': False,
            'mach_polynomial_order': 1,
            'mach_initial': (0.2, 'unitless'),
            'mach_final': (0.72, 'unitless'),
            'mach_bounds': ((0.18, 0.74), 'unitless'),
            'altitude_optimize': False,
            'altitude_polynomial_order': 1,
            'altitude_initial': (0.0, 'ft'),
            'altitude_final': (30500.0, 'ft'),
            'altitude_bounds': ((0.0, 31000.0), 'ft'),
            'throttle_enforcement': 'path_constraint',
            # 'time_initial_bounds': ((0.0, 0.0), 'min'),
            # 'time_duration_bounds': ((27.0, 81.0), 'min'),
        },
        'initial_guesses': {'time': ([0, 54], 'min')},
    },
    'cruise': {
        'subsystem_options': {'core_aerodynamics': {'method': 'computed'}},
        'user_options': {
            'num_segments': 2,
            'order': 3,
            'mach_optimize': False,
            'mach_polynomial_order': 1,
            'mach_initial': (0.72, 'unitless'),
            'mach_final': (0.72, 'unitless'),
            'mach_bounds': ((0.7, 0.74), 'unitless'),
            'altitude_optimize': False,
            'altitude_polynomial_order': 1,
            'altitude_initial': (30500.0, 'ft'),
            'altitude_final': (31000.0, 'ft'),
            'altitude_bounds': ((30000.0, 31500.0), 'ft'),
            'throttle_enforcement': 'boundary_constraint',
            # 'time_initial_bounds': ((27.0, 81.0), 'min'),
            # 'time_duration_bounds': ((85.5, 256.5), 'min'),
        },
        'initial_guesses': {'time': ([54, 171], 'min')},
    },
    'descent_1': {
        'subsystem_options': {'core_aerodynamics': {'method': 'computed'}},
        'user_options': {
            'num_segments': 2,
            'order': 3,
            'mach_optimize': False,
            'mach_initial': (0.72, 'unitless'),
            'mach_final': (0.2, 'unitless'),
            'mach_bounds': ((0.18, 0.74), 'unitless'),
            'altitude_optimize': False,
            'altitude_initial': (31000.0, 'ft'),
            'altitude_final': (500.0, 'ft'),
            'altitude_bounds': ((0.0, 31500.0), 'ft'),
            'throttle_enforcement': 'path_constraint',
            # 'time_initial_bounds': ((112.5, 337.5), 'min'),
            # 'time_duration_bounds': ((26.5, 79.5), 'min'),
        },
        'initial_guesses': {'time': ([171, 225], 'min')},
    },
    'post_mission': {
        'include_landing': False,
        'constrain_range': False,
        'target_range': (2500, 'km'),
    },
}

# prob = av.run_aviary(
#     'models/aircraft/test_aircraft/aircraft_for_bench_FwFm.csv',
#     phase_info,
#     optimizer='SLSQP',
#     make_plots=True,
# )

prob = av.run_aviary(
    'models/aircraft/test_aircraft/crucial_params.csv',
    phase_info,
    optimizer='SLSQP',
    make_plots=True,
)

# """
#     Notes:
# """