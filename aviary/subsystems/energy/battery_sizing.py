import openmdao.api as om

from aviary.utils.aviary_values import AviaryValues
from aviary.variable_info.functions import add_aviary_input, add_aviary_output
from aviary.variable_info.variables import Aircraft, Dynamic

import numpy as np


class StateofCharge(om.ExplicitComponent):
    """Calculates battery energy capacity from pack mass and pack energy density."""
    
    def initialize(self):
        self.options.declare(
            'num_nodes',
            types=int,
            desc='collection of Aircraft/Mission specific options',
        )
        self.options.declare(
            'aviary_inputs',
            types=AviaryValues,
            desc='collection of Aircraft/Mission specific options',
        )

    def setup(self):
        self.num_nodes = self.options['num_nodes']
        add_aviary_input(self, Aircraft.Battery.ENERGY_CAPACITY, val=0.0, units='kJ', desc='mass of energy-storing components of battery')
        add_aviary_input(self, Dynamic.Vehicle.CUMULATIVE_ELECTRIC_ENERGY_USED, val=np.ones(self.num_nodes), units='kJ', desc='energy density of battery pack')
        add_aviary_input(self, Aircraft.Battery.EFFICIENCY, val=0.95, units='unitless', desc='energy density of battery pack')
        add_aviary_output(self, Dynamic.Vehicle.BATTERY_STATE_OF_CHARGE, np.ones(self.num_nodes), units='unitless', desc='total battery energy storage')

    def compute(self, inputs, outputs):
        outputs[Dynamic.Vehicle.BATTERY_STATE_OF_CHARGE] = (inputs[Aircraft.Battery.ENERGY_CAPACITY] - \
            inputs[Dynamic.Vehicle.CUMULATIVE_ELECTRIC_ENERGY_USED] / inputs[Aircraft.Battery.EFFICIENCY])/inputs[Aircraft.Battery.ENERGY_CAPACITY]
        print('BATTERY_STATE_OF_CHARGE: ', outputs[Dynamic.Vehicle.BATTERY_STATE_OF_CHARGE])
        print('EFFICIENCY: ', inputs[Aircraft.Battery.EFFICIENCY])
        print('ENERGY_CAPACITY: ', inputs[Aircraft.Battery.ENERGY_CAPACITY])
        print('CUMULATIVE_ELECTRIC_ENERGY_USED: ', inputs[Dynamic.Vehicle.CUMULATIVE_ELECTRIC_ENERGY_USED])
        
    def setup_partials(self):
        self.declare_partials(Dynamic.Vehicle.BATTERY_STATE_OF_CHARGE, Aircraft.Battery.ENERGY_CAPACITY)
        self.declare_partials(Dynamic.Vehicle.BATTERY_STATE_OF_CHARGE, Dynamic.Vehicle.CUMULATIVE_ELECTRIC_ENERGY_USED)
        self.declare_partials(Dynamic.Vehicle.BATTERY_STATE_OF_CHARGE, Aircraft.Battery.EFFICIENCY)
        
class EnergyCapacity(om.ExplicitComponent):
    """Calculates battery energy capacity from pack mass and pack energy density."""
    
    def initialize(self):
        self.options.declare(
            'aviary_inputs',
            types=AviaryValues,
            desc='collection of Aircraft/Mission specific options',
        )

    def setup(self):
        add_aviary_input(self, Aircraft.Battery.MASS, val=0.0, units='kg', desc='mass of energy-storing components of battery')
        add_aviary_input(self, Aircraft.Battery.PACK_ENERGY_DENSITY, val=0.0, units='kJ/kg', desc='energy density of battery pack')
        add_aviary_output(self, Aircraft.Battery.ENERGY_CAPACITY, val=0.0, units='kJ', desc='total battery energy storage')

    def compute(self, inputs, outputs):
        
        outputs[Aircraft.Battery.ENERGY_CAPACITY] = inputs[Aircraft.Battery.MASS] * inputs[Aircraft.Battery.PACK_ENERGY_DENSITY]
        print('Battery energy capacity: ', outputs[Aircraft.Battery.ENERGY_CAPACITY])
        print('Battery mass: ', inputs[Aircraft.Battery.MASS])
        
    def setup_partials(self):
        self.declare_partials(Aircraft.Battery.ENERGY_CAPACITY, Aircraft.Battery.PACK_ENERGY_DENSITY)
        self.declare_partials(Aircraft.Battery.ENERGY_CAPACITY, Aircraft.Battery.MASS)

class SizeBattery(om.ExplicitComponent):
    """Calculates battery mass from specific energy and additional mass."""

    def initialize(self):
        self.options.declare(
            'aviary_inputs',
            types=AviaryValues,
            desc='collection of Aircraft/Mission specific options',
        )

    def setup(self):
        add_aviary_input(
            self,
            Aircraft.Battery.PACK_MASS,
            val=0.0,
            units='kg',
            desc='mass of energy-storing components of battery',
        )
        add_aviary_input(
            self,
            Aircraft.Battery.ADDITIONAL_MASS,
            val=0.0,
            units='kg',
            desc='mass of non energy-storing components of battery',
        )
        add_aviary_input(
            self,
            Aircraft.Battery.PACK_ENERGY_DENSITY,
            val=0.0,
            units='kJ/kg',
            desc='energy density of battery pack',
        )

        add_aviary_output(
            self, Aircraft.Battery.MASS, val=0.0, units='kg', desc='total battery mass'
        )
        add_aviary_output(
            self,
            Aircraft.Battery.ENERGY_CAPACITY,
            val=0.0,
            units='kJ',
            desc='total battery energy storage',
        )

    def compute(self, inputs, outputs):
        energy_density_kj_kg = inputs[Aircraft.Battery.PACK_ENERGY_DENSITY]
        addtl_mass = inputs[Aircraft.Battery.ADDITIONAL_MASS]
        pack_mass = inputs[Aircraft.Battery.PACK_MASS]
        
        total_mass = pack_mass + addtl_mass
        total_energy = pack_mass * energy_density_kj_kg
        
        outputs[Aircraft.Battery.MASS] = total_mass
        outputs[Aircraft.Battery.ENERGY_CAPACITY] = total_energy

    def setup_partials(self):
        self.declare_partials(
            Aircraft.Battery.ENERGY_CAPACITY, Aircraft.Battery.PACK_ENERGY_DENSITY
        )
        self.declare_partials(Aircraft.Battery.ENERGY_CAPACITY, Aircraft.Battery.PACK_MASS)

        self.declare_partials(Aircraft.Battery.MASS, Aircraft.Battery.ADDITIONAL_MASS, val=1.0)
        self.declare_partials(Aircraft.Battery.MASS, Aircraft.Battery.PACK_MASS, val=1.0)

    def compute_partials(self, inputs, J):
        energy_density_kj_kg = inputs[Aircraft.Battery.PACK_ENERGY_DENSITY]
        pack_mass = inputs[Aircraft.Battery.PACK_MASS]

        J[Aircraft.Battery.ENERGY_CAPACITY, Aircraft.Battery.PACK_ENERGY_DENSITY] = pack_mass
        J[Aircraft.Battery.ENERGY_CAPACITY, Aircraft.Battery.PACK_MASS] = energy_density_kj_kg
