"""Motor, inverter and controller parameters.

The default motor is an ILLUSTRATIVE small surface-mount PMSM of the 24 V,
4000 rpm class used in many lab drives. Replace these values with your own
motor's datasheet or measured values before quoting any result for a specific
machine.
"""
from dataclasses import dataclass, field
import math


@dataclass
class MotorParams:
    p: int = 4                 # pole pairs
    Rs: float = 0.36           # stator resistance per phase [ohm]
    Ls: float = 0.6e-3         # synchronous inductance, Ld = Lq [H] (surface-mount)
    psi_f: float = 0.0064      # PM flux linkage [Wb]
    J: float = 4.8e-5          # rotor + coupled load inertia [kg m^2] (about 10x the bare rotor)
    B: float = 1.0e-5          # viscous friction [N m s/rad]

    @property
    def Kt(self) -> float:
        """Torque constant [N m/A] for id = 0: Te = 1.5 p psi_f iq."""
        return 1.5 * self.p * self.psi_f


@dataclass
class InverterParams:
    Vdc: float = 24.0          # DC-link voltage [V]

    @property
    def Vmax(self) -> float:
        """Largest phase-voltage amplitude in the linear SVPWM range [V]."""
        return self.Vdc / math.sqrt(3.0)


@dataclass
class ControlParams:
    fs: float = 20e3           # control (PWM) frequency [Hz]
    substeps: int = 5          # plant integration steps per control period
    i_max: float = 5.0         # current limit [A]
    current_bw_hz: float = 1000.0   # current-loop bandwidth
    speed_bw_hz: float = 20.0       # speed-loop bandwidth
    # I/f open-loop start-up
    align_time: float = 0.05        # rotor alignment before the I/f ramp [s]
    if_current: float = 3.0         # current amplitude during start-up [A]
    if_accel_rpm_s: float = 3000.0  # start-up speed ramp [rpm/s]
    switch_rpm: float = 400.0       # hand over to the estimator above this speed

    @property
    def Ts(self) -> float:
        return 1.0 / self.fs


@dataclass
class SMOParams:
    # Inside the boundary layer the observer is linear with gain k/phi [V/A].
    # Forward-Euler stability at the control rate needs Ts/Ls*(Rs + k/phi) < 1
    # (here k/phi < ~11.6). Above that the observer error alternates sign every
    # sample and the speed estimate becomes unusable at low speed.
    k: float = 24.0            # switching gain [V], must exceed peak back-EMF (~10.7 V at 4000 rpm)
    phi: float = 2.5           # boundary layer [A] -> k/phi = 9.6 V/A
    lpf_wc: float = 2500.0     # back-EMF low-pass cut-off [rad/s]
    pll_bw_hz: float = 80.0    # PLL natural frequency
    pll_zeta: float = 1.0


@dataclass
class MRASParams:
    kp: float = 80.0           # adaptation gains, tuned by hand on E1-E3
    ki: float = 4.0e4


@dataclass
class Config:
    motor: MotorParams = field(default_factory=MotorParams)
    inverter: InverterParams = field(default_factory=InverterParams)
    control: ControlParams = field(default_factory=ControlParams)
    smo: SMOParams = field(default_factory=SMOParams)
    mras: MRASParams = field(default_factory=MRASParams)
    current_noise_std: float = 0.0  # optional ADC noise on measured currents [A]
    seed: int = 1
