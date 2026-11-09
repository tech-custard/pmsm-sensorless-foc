# Theory notes: what every line of the code is doing

These notes are written so you can explain the project on a call with a drives professor without looking at the code. Each section ends with the questions you should be able to answer.

## 1. PMSM model in the rotor frame

For a surface-mount PMSM (Ld = Lq = Ls), in the d-q frame aligned with the magnet flux:

```
vd = Rs id + Ls did/dt - we Ls iq
vq = Rs iq + Ls diq/dt + we Ls id + we psi_f
Te = 1.5 p psi_f iq
J dwm/dt = Te - B wm - TL,      we = p wm,  dtheta_e/dt = we
```

The term `we psi_f` is the back-EMF. It is proportional to speed, which is the root of every low-speed problem in back-EMF-based sensorless control. Because Ld = Lq there is no reluctance torque, so id = 0 gives maximum torque per ampere.

Code: `pmsm_foc/motor.py` integrates exactly these equations with RK4 at 5 sub-steps per control period.

**Be ready for:** Why is id set to zero? What changes for an interior PMSM (Ld < Lq)? Why amplitude-invariant transforms give the 1.5 factor.

## 2. Current loops

The current dynamics are first order, `Ls di/dt + Rs i = v`. A PI controller with `Kp = Ls*wc` and `Ki = Rs*wc` cancels the electrical pole at Rs/Ls, so the closed loop becomes `wc/(s + wc)`, a first-order response with bandwidth `wc`. The cross-coupling terms (`-we Ls iq`, `+we (Ls id + psi_f)`) are added as feed-forward so each axis behaves like an isolated RL circuit.

Bandwidth choice: 1 kHz, a twentieth of the 20 kHz control rate, which keeps the discretisation effects small.

**Be ready for:** What happens if Ls in the controller is wrong by 20%? (The cancellation is imperfect, so the response gains a slow tail but stays stable.) Why limit the voltage vector to Vdc/sqrt(3)?

## 3. Speed loop

With the current loop much faster than the speed loop, the plant seen by the speed PI is `Kt/(J s)`. Choosing `Kp = J ws/Kt` and `Ki = Kp ws/4` places the crossover near `ws` (20 Hz) with good damping. Anti-windup stops the integrator charging while the current is limited at 5 A.

## 4. Start-up: why sensorless drives need it

At standstill there is no back-EMF, so neither estimator can see the rotor. The drive therefore:

1. **Aligns** the rotor by injecting 3 A on the d-axis at angle 0 for 50 ms.
2. **Ramps I/f**: the same current vector rotates open-loop at increasing frequency. The rotor follows with a load angle delta such that `Te = Kt * I * sin(delta)`.
3. **Hands over** at 400 rpm. The speed integrator is preset to `I sin(delta)`, so the torque does not jump. The leftover d-axis current `I cos(delta)` is ramped to zero over 50 ms.

An earlier version started I/f with the current on the q-axis and no alignment. That put the rotor at a 90 degree load angle at t = 0, so it surged and oscillated before handover.

## 5. Sliding-mode observer (SMO) with PLL

Stationary-frame model: `Ls di/dt = v - Rs i - e`, where `e = we psi_f [-sin(theta), cos(theta)]`.

Observer: `Ls di_hat/dt = v - Rs i_hat - z`, with `z = k sat((i_hat - i)/phi)`.

When the observer slides (`i_hat = i`), the switching term `z` equals the back-EMF on average. A low-pass filter at `wc_f = 2500 rad/s` gives `e_hat`, which lags by `atan(we/wc_f)`. That lag is added back to the angle.

The PLL drives `err = -e_alpha cos(theta_hat) - e_beta sin(theta_hat)`, which is proportional to `sin(theta - theta_hat)`, to zero. Dividing by `|e_hat|` keeps the PLL gain constant across speed. Its integrator output is used as the speed feedback because it is smoother than the proportional part.

### The discrete stability condition (the bug worth explaining)

Inside the boundary layer the observer is linear with gain `k/phi`. With forward-Euler at step `Ts`, the error obeys

```
e[n+1] = (1 - Ts/Ls (Rs + k/phi)) e[n]
```

If `Ts/Ls (Rs + k/phi) > 1`, the pole is negative: the error flips sign every sample. With `k = 30 V`, `phi = 1.5 A` the pole was about -0.70. At high speed the large back-EMF masked it. At low speed and under load the PLL locked onto the ringing and the drive lost control. The chosen `k = 24 V`, `phi = 2.5 A` gives `k/phi = 9.6 V/A` and a pole of about +0.17.

**Be ready for:** Why must `k` exceed the peak back-EMF? Why does the SMO fail near zero speed (E3 at 50 rpm)? What does the boundary layer trade away (chattering against tracking accuracy)?

## 6. MRAS

Shift the d-axis current by the magnet term, `i'_d = id + psi_f/Ls`, so the rotor-frame current model becomes

```
d/dt [i'_d]   [-Rs/Ls   we  ] [i'_d]          [v'_d]
     [i'_q] = [ -we   -Rs/Ls] [i'_q] + 1/Ls  [ vq ],     v'_d = vd + Rs psi_f/Ls
```

The motor (measured currents) is the reference model. The same equations run with the estimated speed as the adjustable model. Popov hyperstability gives the adaptation law

```
eps = i'_d * i'_q_hat - i'_q * i'_d_hat
w_hat = Kp eps + Ki * integral(eps)
```

The rotor angle is the integral of `w_hat`. If the angle is wrong, the measured d-q currents are wrong too, which drives `eps` and corrects the speed.

**Why it is fragile at low speed:** the adjustable model contains Rs and psi_f directly. At low speed the `Rs i` voltage drop is comparable to the back-EMF, so a resistance error is a large relative error. E4 shows this: 50% Rs error gives about 32 degrees of angle error at 300 rpm, and a 10% psi_f error loses lock.

**Be ready for:** Derive the adaptation law from the error dynamics. Why does Rs sensitivity grow as speed falls? How would you add online Rs estimation (a second adaptation law driven by the d-axis error)?

## 7. What the comparison says, honestly

- With exact parameters both estimators work well from 150 rpm up. The MRAS even holds at 50 rpm.
- The SMO is much more robust to parameter error, because it only needs Rs and Ls for the current model and extracts the back-EMF directly.
- The SMO is less noisy with current-sensor noise in this tuning.
- None of this includes PWM ripple, dead time or saturation, so absolute numbers will change on hardware. The ranking under parameter error is the result most likely to survive.

## 8. Questions a professor is likely to ask

1. Why does sensorless control based on back-EMF fail at zero speed, and what do people use instead? (High-frequency injection for salient machines.)
2. How did you choose the PLL bandwidth?
3. What is the effect of the back-EMF filter, and how do you compensate it?
4. How would you tune these estimators on a real STM32 at 20 kHz? (Fixed-point or float, computation time per step, ADC timing.)
5. Your thesis model was an induction-motor derivation. What did you change for the PMSM? (Section 1 here.)
6. What would you measure first on the hardware? (Rs by DC test, Ls by step response, psi_f from open-circuit back-EMF.)
