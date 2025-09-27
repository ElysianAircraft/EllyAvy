"""Run the a mission with a simple external component that computes aircraft lift and drag."""

from copy import deepcopy

import aviary.api as av
from aviary.examples.external_subsystems.custom_aero.custom_aero_builder import CustomAeroBuilder
from aviary.api import Aircraft, Mission, Settings, ProblemType
from aviary.subsystems.energy.battery_builder import BatteryBuilder


# Just do cruise in this example.
# phase_info.pop('climb')
# phase_info.pop('descent')

oswald_factor = 0.08
aspect_ratio = 12
CD0 = 0.01
k = 0.02 # 1/(np.pi*aspect_ratio*oswald_factor)

phase_info = {
    'pre_mission': {
                'include_takeoff': False,
                'external_subsystems': [BatteryBuilder()],
                'optimize_mass': True,
            },
    'climb': {
        'external_subsystems': [
            CustomAeroBuilder(),
            BatteryBuilder()
            ],
        'subsystem_options': {'core_aerodynamics': {'method': 'external'}},
        'user_options': {
            'num_segments': 5,
            'order': 3,
            'mach_optimize': False,
            'mach_initial': (0.2, 'unitless'),
            'mach_final': (0.72, 'unitless'),
            'mach_bounds': ((0.18, 0.74), 'unitless'),
            'mach_polynomial_order': 3,
            'altitude_optimize': False,
            'altitude_initial': (0.0, 'ft'),
            'altitude_final': (32000.0, 'ft'),
            'altitude_bounds': ((0.0, 34000.0), 'ft'),
            'altitude_polynomial_order': 3,
            'throttle_enforcement': 'path_constraint',
            'time_initial_bounds': ((0.0, 0.0), 'min'),
            'time_duration_bounds': ((64.0, 192.0), 'min'),
        },
    },
    'cruise': {
        'external_subsystems': [
            CustomAeroBuilder(),
            BatteryBuilder()
            ],
        'subsystem_options': {'core_aerodynamics': {'method': 'external'}},
        'user_options': {
            'num_segments': 5,
            'order': 3,
            'mach_optimize': False,
            'mach_initial': (0.72, 'unitless'),
            'mach_final': (0.72, 'unitless'),
            'mach_bounds': ((0.7, 0.74), 'unitless'),
            'mach_polynomial_order': 3,
            'altitude_optimize': False,
            'altitude_initial': (32000.0, 'ft'),
            'altitude_final': (34000.0, 'ft'),
            'altitude_bounds': ((23000.0, 38000.0), 'ft'),
            'altitude_polynomial_order': 3,
            'throttle_enforcement': 'boundary_constraint',
            'time_initial_bounds': ((64.0, 192.0), 'min'),
            'time_duration_bounds': ((56.5, 300.0), 'min'),
        },
    },
    'descent': {
        'external_subsystems': [
            CustomAeroBuilder(),
            BatteryBuilder()
            ],
        'subsystem_options': {'core_aerodynamics': {'method': 'external'}},
        'user_options': {
            'num_segments': 5,
            'order': 3,
            'mach_optimize': False,
            'mach_initial': (0.72, 'unitless'),
            'mach_final': (0.36, 'unitless'),
            'mach_bounds': ((0.34, 0.74), 'unitless'),
            'mach_polynomial_order': 3,
            'altitude_optimize': False,
            'altitude_initial': (34000.0, 'ft'),
            'altitude_final': (500.0, 'ft'),
            'altitude_bounds': ((0.0, 38000.0), 'ft'),
            'altitude_polynomial_order': 3,
            'throttle_enforcement': 'path_constraint',
            'time_initial_bounds': ((120.5, 361.5), 'min'),
            'time_duration_bounds': ((29.0, 87.0), 'min'),
        },
    },
    'post_mission': {
        'include_landing': False,
        'include_batteries': True,
        'constrain_range': True,
        'target_range': (1906.0, 'nmi'),
    },
}


# Add custom aero.
# TODO: This API for replacing aero will be changed an upcoming release.
# phase_info['cruise']['external_subsystems'] = [CustomAeroBuilder()]

# # Disable internal aero
# # TODO: This API for replacing aero will be changed an upcoming release.
# phase_info['cruise']['subsystem_options']['core_aerodynamics'] = {
#     'method': 'external',
# }


if __name__ == '__main__':
    prob = av.AviaryProblem()
    
    # Load aircraft and options data from user
    # Allow for user overrides here
    prob.load_inputs('models/aircraft/test_aircraft/crucial_params.csv', phase_info)

    # prob.load_inputs('models/aircraft/test_aircraft/aircraft_for_bench_FwFm.csv', phase_info)
    
    # Preprocess inputs
    prob.check_and_preprocess_inputs()

    prob.add_pre_mission_systems()

    prob.add_phases()

    prob.add_post_mission_systems()

    # Link phases and variables
    prob.link_phases()

    # Note, SLSQP has trouble here.
    prob.add_driver('SLSQP')

    prob.add_design_variables()

    prob.add_objective()

    prob.setup()
    
    prob.model.set_val(Aircraft.CrewPayload.TOTAL_PAYLOAD_MASS, 20_000, units='lbm')
    prob.model.set_val(Aircraft.Design.OPERATING_MASS, 175_400-80_000, units='lbm')
    prob.set_val(av.Aircraft.Battery.PACK_ENERGY_DENSITY, 360*3.6, units='kJ/kg')
    prob.set_val(av.Aircraft.Battery.PACK_MASS, 28_000, units='lbm')

    prob.set_initial_guesses()

    prob.run_aviary_problem(suppress_solver_print=False)
    
    print(f"RANGE: {prob[Mission.Summary.RANGE][0]:.0f} km")
    print(f"TOTAL_PAYLOAD_MASS: {prob[Aircraft.CrewPayload.TOTAL_PAYLOAD_MASS][0]:.0f} lbm")
    print(f"TOTAL_FUEL_MASS: {prob[Mission.Summary.TOTAL_FUEL_MASS][0]:.0f} lbm")
    print(f"TOTAL_BATTERY_MASS: {prob[Mission.Summary.TOTAL_BATTERY_MASS][0]:.0f} lbm")
    print(f"OPERATING_MASS: {prob[Aircraft.Design.OPERATING_MASS][0]:.0f} lbm")
    print(f"GROSS_MASS: {prob[Mission.Design.GROSS_MASS][0]:.0f} lbm")
    
    # print(f"BATTERY_SoC: {prob['traj.cruise.timeseries.battery_state_of_charge']:.0f} %")
    print(prob['traj.cruise.timeseries.battery_state_of_charge'])
    print(prob['traj.descent.timeseries.battery_state_of_charge'])

    print('done')
