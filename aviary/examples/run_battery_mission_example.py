"""Example mission using the a detailed battery model."""

from aviary.api import default_height_energy_phase_info as phase_info
from aviary.subsystems.energy.battery_builder import BatteryBuilder
from aviary.interface.methods_for_level2 import AviaryProblem
from aviary.utils.functions import get_aviary_resource_path
from aviary.variable_info.variables import Aircraft, Dynamic, Mission

phase_info = {
    'pre_mission': {'include_takeoff': False, 'optimize_mass': True},
    'climb': {
        'subsystem_options': {'core_aerodynamics': {'method': 'computed'}},
        'user_options': {
            'num_segments': 5,
            'order': 3,
            'mach_optimize': False,
            'mach_initial': (0.2, 'unitless'),
            'mach_final': (0.5, 'unitless'),
            'mach_bounds': ((0.18, 0.74), 'unitless'),
            'mach_polynomial_order': 3,
            'altitude_optimize': False,
            'altitude_initial': (0.0, 'ft'),
            'altitude_final': (28_000.0, 'ft'),
            'altitude_bounds': ((0.0, 34000.0), 'ft'),
            'altitude_polynomial_order': 3,
            'throttle_enforcement': 'path_constraint',
            'time_initial': (0.0, 'min'),
            'time_duration_bounds': ((10.0, 180.0), 'min'),
        },
    },
    'cruise': {
        'subsystem_options': {'core_aerodynamics': {'method': 'computed'}},
        'user_options': {
            'num_segments': 5,
            'order': 3,
            'mach_optimize': False,
            'mach_initial': (0.5, 'unitless'),
            'mach_final': (0.5, 'unitless'),
            'mach_bounds': ((0.7, 0.74), 'unitless'),
            'mach_polynomial_order': 3,
            'altitude_optimize': False,
            'altitude_initial': (28_000.0, 'ft'),
            'altitude_final': (28_000.0, 'ft'),
            'altitude_bounds': ((23000.0, 38000.0), 'ft'),
            'altitude_polynomial_order': 3,
            'throttle_enforcement': 'path_constraint',
            'time_initial_bounds': ((10.0, 180.0), 'min'),
            'time_duration_bounds': ((10.0, 180.0), 'min'),
        },
    },
    'descent': {
        'subsystem_options': {'core_aerodynamics': {'method': 'computed'}},
        'user_options': {
            'num_segments': 5,
            'order': 3,
            'mach_optimize': False,
            'mach_initial': (0.5, 'unitless'),
            'mach_final': (0.36, 'unitless'),
            'mach_bounds': ((0.34, 0.74), 'unitless'),
            'mach_polynomial_order': 3,
            'altitude_optimize': False,
            'altitude_initial': (28_000.0, 'ft'),
            'altitude_final': (500.0, 'ft'),
            'altitude_bounds': ((0.0, 38000.0), 'ft'),
            'altitude_polynomial_order': 3,
            'throttle_enforcement': 'path_constraint',
            'time_initial_bounds': ((20.0, 190.0), 'min'),
            'time_duration_bounds': ((10.0, 180.0), 'min'),
        },
    },
    'post_mission': {
        'include_landing': False,
        'constrain_range': True,
        'target_range': (900.0, 'nmi'),
    },
}

battery_builder = BatteryBuilder()

# add the battery model to each mission phase, as well as pre-mission for sizing
phase_info['pre_mission']['external_subsystems'] = [battery_builder]
phase_info['climb']['external_subsystems'] = [battery_builder]
phase_info['cruise']['external_subsystems'] = [battery_builder]
phase_info['descent']['external_subsystems'] = [battery_builder]


if __name__ == '__main__':
    prob = AviaryProblem()

    # Load aircraft and options data from user
    # Allow for user overrides here
    input_file = get_aviary_resource_path(
        'models/aircraft/test_aircraft/aircraft_for_bench_FwFm_with_electric.csv'
    )
    prob.load_inputs(input_file, phase_info)

    prob.check_and_preprocess_inputs()

    prob.build_model()

    prob.add_driver('SLSQP')

    prob.add_design_variables()

    prob.add_objective('mass',
                       ref=1)
    # prob.model.add_objective(
    #     f'traj.climb.states:{Dynamic.Battery.STATE_OF_CHARGE}', index=-1, ref=-1)
    
    prob.setup()
    
    # prob.driver.options['debug_print'] = ['desvars']

    prob.model.set_val(Aircraft.Battery.PACK_MASS, 30_000, units='lbm')
    prob.model.set_val(Aircraft.Battery.PACK_ENERGY_DENSITY, 320*3.6*0.453, units='kJ/lbm')
    
    prob.run_aviary_problem()
    # prob.run_model()
    # prob.check_totals(compact_print=True, show_only_incorrect=True)

    import numpy as np
    
    M_gross = prob.get_val(Mission.Summary.GROSS_MASS)
    M_OEM = prob.get_val(Aircraft.Design.OPERATING_MASS)
    M_PL = prob.get_val(Aircraft.CrewPayload.TOTAL_PAYLOAD_MASS)
    M_BAT = prob.get_val(Aircraft.Battery.MASS)
    M_F = prob.get_val(Mission.Summary.TOTAL_FUEL_MASS)
    
    final_phase_name = 'descent'

    print("Range:",np.round(prob.get_val(Mission.Summary.RANGE), 2))
    print("Battery State of Charge:",np.round(prob.get_val('traj.descent.timeseries.battery_state_of_charge'), 2))
        
    print("Timeseries mass:",np.round(prob.get_val(f'traj.{final_phase_name}.timeseries.{Dynamic.Vehicle.MASS}'), 2))
    print("M_gross:",np.round(M_gross, 2))
    print("M_PL:",np.round(M_PL, 2))
    print("M_OEM:",np.round(M_OEM, 2))
    print("M_BAT:",np.round(M_BAT, 2))
    print("M_F:",np.round(M_F, 2))
    print("M_gross - (M_PL + M_OEM + M_F):",np.round(M_gross - (M_PL + M_OEM + M_F), 2))
    print("Gross Mass:",np.round(prob.get_val(Mission.Summary.GROSS_MASS), 2))
