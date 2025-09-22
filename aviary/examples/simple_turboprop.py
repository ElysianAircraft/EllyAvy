import aviary.api as av
from aviary.models.aircraft.large_turboprop_freighter.phase_info import two_dof_phase_info, energy_phase_info

Aircraft = av.Aircraft
Mission = av.Mission
Dynamic = av.Dynamic

# define the minimum option set for a turboprop
options = av.AviaryValues()

# top-level turboprop settings
options.set_val(av.Settings.VERBOSITY, 0)  # quiet unneeded printouts
options.set_val(Aircraft.Engine.FIXED_RPM, 13820, units='rpm')

# EngineDeck minimum option set
options.set_val(Aircraft.Engine.DATA_FILE, av.get_path('models/engines/turboshaft_4465hp.csv'))

# Gearbox model minimum option set
options.set_val(Aircraft.Engine.Gearbox.GEAR_RATIO, 13.55, 'unitless')
options.set_val(Aircraft.Engine.Gearbox.SHAFT_POWER_DESIGN, 4465, 'hp')

# Hamilton Standard propeller minimum option set
options.set_val(Aircraft.Engine.Propeller.TIP_MACH_MAX, 1.0)
options.set_val(Aircraft.Engine.Propeller.NUM_BLADES, val=4, units='unitless')
options.set_val(Aircraft.Engine.Propeller.COMPUTE_INSTALLATION_LOSS, True)

# Initialize turboprop model. Model uses an EngineDeck built from `options`, basic
# gearbox model with default efficiency of 1, and the Hamilton Standard propeller model
# "turboprop" is ready to be included in an AviaryProblem
turboprop = av.TurbopropModel(name='turboprop_example', options=options)

# Build and run AviaryProblem using the Level2 interface
prob = av.AviaryProblem()

prob.load_inputs(
    'models/aircraft/large_turboprop_freighter/large_turboprop_freighter_GASP.csv',
    two_dof_phase_info,
    engine_builders=[turboprop],
)

prob.check_and_preprocess_inputs()
prob.add_pre_mission_systems()
prob.add_phases()
prob.add_post_mission_systems()
prob.link_phases()
prob.add_driver('SLSQP', max_iter=100, verbosity=0)
prob.add_design_variables()
prob.add_objective()
prob.setup()

prob.set_initial_guesses()
prob.run_aviary_problem(suppress_solver_print=True, make_plots=True)