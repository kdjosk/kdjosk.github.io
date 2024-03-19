---
date: 2024-02-13
layout: "post.jinja"
title: 'How to land a rocket safely'
tags: ['post', 'rocket-landing']
summary: "Pratical introduction to the G-FOLD algorithm"
---

During my time at the university I was trying to implement a controller with [kRPC](https://krpc.github.io/krpc/) for Kerbal Space Program that would allow me to safely land a rocket, but would be a bit smarter than simple PID regulators. I've then come across an algorithm called G-FOLD and its open source implementations:

- [HopperGuidance](https://github.com/oyster-catcher/HopperGuidance)
- [G-FOLD-Python](https://github.com/jonnyhyman/G-FOLD-Python)

They both are based on the paper titled [_"Lossless Convexification of Nonconvex Control
Bound and Pointing Constraints of the Soft
Landing Optimal Control Problem"_](http://larsblackmore.com/iee_tcst13.pdf). In the first part of this series I'd like to introduce the algorithm and present a simple implementation using the Python API of [CasADi](https://web.casadi.org/), which is an open-source tool for defining and solving nonlinear optimization problems that provides algorithmic differentiation (so you don't have to calculate Jacobians and Hessians of the yourself). There are two consecutive optimization problems presented in this paper. The first one tries to land the rocket as close to the target landing point \\(\mathbf{q}\\) as possible, thus returning some optimal landing point \\(\mathbf{d}^\*\_{P1}\\) The second one tries to improve on the previous solution to use as little fuel as possible while landing not further from \\(\mathbf{q}\\) than \\(\mathbf{d}^\*\_{P1}\\).

## Engine characteristics

The main characteristic of a rocket engine is specific impulse \\(I_{sp}\\). To explain what it means in simple terms: it's a measure of how effectively the engine converts the propellant to the thrust force. To derive it, suppose we have a specific engine with a specific propellant. The engine ejects \\(dm\\) kilograms of propellant in \\(dt\\) seconds. Therefore \\(\dot{m} = dm / dt\\) gives us the mass flow rate. The engine also exerts a given thrust force \\(F\\). We then define the initial propellant mass \\(m_0 \\) as \\(m_0 = F/g_0\\). The time that the engine takes to burn through all of this propellant is the specific impulse:

$$
I_{sp} = \frac{F}{g_0 \dot{m}}
$$

The authors of the paper then define the \\(\alpha\\) parameter as a function of the specific impulse 

$$
\alpha = \frac{1}{I_{sp}g_0}
$$

## Lander model

For the purpose of formulating the model, the authors of the paper treat the vehicle as a point mass. Therefore the model consists of position and velocity in 3D space, omitting orientation.

$$
\mathbf{X} = (\mathbf{r}, \mathbf{\dot{r}}) \in \mathbb{R}^6
$$

The 7th state variable is mass \\(m\\). The vehicle is being controlled only by the thrust force \\(\mathbf{T}_c\\). Therefore we can write the model equations as:

$$\begin{align}
\dot{\mathbf{x}}(t) & = \begin{bmatrix}
    \mathbf{\dot{r}}(t)\\\\
    \mathbf{g} + \frac{\mathbf{T}_c(t)}{m}
\end{bmatrix} \\\\
\dot{m}(t) & = -\alpha \|\mathbf{T}_c(t)\| \\\\
\end{align}$$

The authors of the paper use a helper matrix \\(\mathbf{A}\\) and a helper vector \\(\mathbf{B}\\) to write this equation:

$$\begin{align}
\dot{\mathbf{x}}(t) & = \mathbf{A}\mathbf{x}(t) + \mathbf{B}\left(\mathbf{g} + \frac{\mathbf{T}_c(t)}{m}\right) \\\\
\dot{m}(t) & = -\alpha \|\mathbf{T}_c(t)\| \\\\
\end{align}$$

Where

$$
\mathbf{A} = \begin{bmatrix}
    \mathbf{0}\_{3\times3} & \mathbf{I}\_{3\times3} \\\\
    \mathbf{0}\_{3\times3} & \mathbf{0}\_{3\times3}
\end{bmatrix}, \quad
\mathbf{B} = \begin{bmatrix}
    \mathbf{0}\_{3\times3} \\\\ \mathbf{I}\_{3\times3}
\end{bmatrix}
$$

I've made one simplification with regards to the model presented in the paper, which is not taking the rotation of the planet into account.

## Optimization constraints

First let's summarize what we're given:

- \\(\mathbf{r}_0\\) - initial position
- \\(\dot{\mathbf{r}}_0\\) - initial velocity
- \\(\mathbf{g}\\) - vector of gravitational acceleration
- \\(m_0\\) - inital mass of the vehicle
- \\(m_f\\) - initial mass of the propellant
- \\(\theta\\) - max angle of thrust vector w.r.t the vertical direction
- \\(\alpha\\) - parameter characterizing the engine
- \\(\rho_1\\) - lower bound for thrust magnitude (rocket engines don't operate from 0% to 100%, but e.g. from 60% to 100%)
- \\(\rho_2\\) - upper bound for thrust magnitude


The authors also define the set of all feasible positions and velocities accordingly:

$$
\mathbf{X} = \{ (\mathbf{r}, \mathbf{\dot{r}}) \in \mathbb{R}^6 \, : \, \|\mathbf{\dot{r}} \| \leq V_{max}, \; \|E(\mathbf{r} - \mathbf{r}(t_f))\| - \mathbf{c}^T(\mathbf{r} - \mathbf{r}(t_f)) \leq 0 \} 
$$

where \\(\mathbf{c}\\) defines a cone with it's point in \\(\mathbf{r}(t_f)\\), parametrized with the glide slope angle \\(\gamma_{gs}\\)

$$
\mathbf{c} \triangleq \frac{\mathbf{e}\_1}{\tan{\gamma_{gs}}}, \quad \gamma_{gs} \in (0,\, \pi / 2).
$$

The \\(\mathbf{X}\\) set is used to define a constraint on the state variable \\(\mathbf{x}(t)\\)

$$
\mathbf{x}(t) \in \mathbf{X} \quad \forall t \in [0, t_f]
$$

Then for the hard part, the constraints on thrust. They consist of lower and upper bounds for magnitude and pointing constraint for direction. The thrust is a 3D vector, therefore the upper bound constraint \\(\|\mathbf{T}_c\| \leq \rho_2 \\) defines a ball. It's shown on the diagram in blue. The lower bound constraint \\(0 \lt \rho_1 \leq \|\mathbf{T}_c\| \\) cuts out a smaller ball from the bigger one, making the constraint a non-convex shape. It's shown on the diagram in red. Lastly, the thrust pointing constraint \\( \hat{\mathbf{n}}^T\mathbf{T}_c(t) \geq \|\mathbf{T}_c\|\cos{\theta}\\) is a cone, shown on the diagram in green. The vertical direction normal vector \\(\hat{\mathbf{n}} = \begin{bmatrix} 1 & 0 & 0 \end{bmatrix}^T\\) points along the x axis. This constraint prohibits the lander from changing the attitude too much. In the paper you can find a 2D cross-section of this diagram.

<img src="/static/thrust_constraint.svg" alt="Thrust constraints" style="width:90%;"/>

## Nonconvex optimization problem

Taking the model and the constraints defined in previous sections into account, the authors of the paper formulate the 1st iteration of the two problems, which is unfortunately non-convex, thus not guaranteed to be solved by interior point numerical methods.

$$\begin{align}
    \min_{t_f, \mathbf{T}_c} \quad & \| E\mathbf{r}(t_f) - \mathbf{q}\| \\\\
    \textrm{s.t.} \quad & \dot{\mathbf{x}}(t) = \mathbf{A}\mathbf{x}(t) + \mathbf{B}\left(\mathbf{g} + \frac{\mathbf{T}_c(t)}{m}\right) \quad \forall t \in [0, t_f] \\\\
    & \dot{m}(t) = -\alpha \|\mathbf{T}_c(t)\| \\\\
    & \mathbf{x}(t) \in \mathbf{X} \quad \forall t \in [0, t_f] \\\\
    & 0 < \rho_1 \leq \|\mathbf{T}_c\| \leq \rho_2, \quad \hat{\mathbf{n}}^T\mathbf{T}_c(t) \geq \|\mathbf{T}_c\| \cos{\theta} \\\\
    & m(0) = m_0, \quad m(t_f) \geq m_0 - m_f > 0 \\\\
    & \mathbf{r}(0) = \mathbf{r}_0, \enspace \dot{\mathbf{r}}(0) = \dot{\mathbf{r}}_0 \\\\
    & \mathbf{e}^T_1\mathbf{r}(t_f) = 0, \enspace \dot{\mathbf{r}}(tf) = \mathbf{0} \\\\
\end{align}$$

And the second problem, which uses the solution from the first problem as a starting point and aims to optimize fuel usage

$$\begin{align}
    \min_{t_f, \mathbf{T}_c} \quad & \int\_{0}^{t_f} \alpha\|\mathbf{T}_c(t)\|dt \\\\
    \textrm{s.t.}\quad & \textrm{dynamics and constraints in problem 1.} \\\\
    & \|E\mathbf{r}(t_f) - \mathbf{q}\| \leq \|\mathbf{d}^*\_{P1} - \mathbf{q}\| \\\\
\end{align}$$

## Relaxation of the nonconvex thrust constraints 

As mentioned before, the solution space is non-convex, therefore the authors propose adding an additional dimension to form a 4D \\(\mathbf{T}_c-\Gamma\\) solution space, which is designed to be convex. The new \\(\Gamma(t)\\) variable is used to pose the following constraints on the thrust vector

1. convex upper bound constraint \\(\|\mathbf{T}_c(t)\| \leq \Gamma(T)\\)
2. convex thrust pointing constraint \\(\hat{\mathbf{n}}^T\mathbf{T}_c(t) \geq \Gamma(t)\cos{\theta}\\). (compare with the previous version: \\(\hat{\mathbf{n}}^T\mathbf{T}_c(t) \geq \|\mathbf{T}_c(t) \|\cos{\theta}\\)). You can find detailed diagrams of how it works in the paper, but to summarize, it defines a half space ... 
3. \\(\rho_1 \leq \Gamma(t) \leq \rho_2\\)


<img src="/static/relaxed_thrust_constraint.svg" alt="Relaxed thrust constraints" style="width:90%;"/>

The full form of the optimization problem is given as 

$$\begin{align}
    \min_{t_f, \mathbf{T}_c, \Gamma} \quad & \| E\mathbf{r}(t_f) - \mathbf{q}\| \\\\
    \textrm{p.o.} \quad & \mathbf{x}(t) \in \mathbf{X} \quad \forall t \in [0, t_f] \\\\
    & 0 < \rho_1 \leq \|\mathbf{T}_c\| \leq \rho_2, \quad \hat{\mathbf{n}}^T\mathbf{T}_c(t) \geq \|\mathbf{T}_c\| \cos{\theta} \\\\
    & m(0) = m_0, \quad m(t_f) \geq m_0 - m_f > 0 \\\\
    & \mathbf{r}(0) = \mathbf{r_0}, \enspace \dot{\mathbf{r}}(0) = \dot{\mathbf{r}}_0 \\\\
    & \mathbf{e}^T_1\mathbf{r}(t_f) = 0, \enspace \dot{\mathbf{r}}(tf) = \mathbf{0} \\\\
    & \dot{\mathbf{x}}(t) = \mathbf{A}\mathbf{x}(t) + \mathbf{B}\left(\mathbf{g} + \frac{\mathbf{T}_c(t)}{m}\right) \quad \forall t \in [0, t_f] \\\\
    & \dot{m}(t) = -\alpha \Gamma(t) \\\\
    & \|\mathbf{T}_c(t)\| \leq \Gamma(t), \enspace 0 < \rho_1 \leq \Gamma(t) \leq \rho_2 \\\\
    & \hat{\mathbf{n}}^T\mathbf{T}_c(t) \geq \cos{\theta} \Gamma(t)
\end{align}$$

Helper vectors and matrices

$$\begin{align}
\mathbf{E} = \begin{bmatrix}
    0 & 1 & 0 \\\\
    0 & 0 & 1
\end{bmatrix}, \quad
\mathbf{A} = \begin{bmatrix}
    \mathbf{0} & \mathbf{I} \\\\
    \mathbf{0} & \mathbf{0}
\end{bmatrix}, \quad
\mathbf{B} = \begin{bmatrix}
    \mathbf{0} \\\\ \mathbf{I}
\end{bmatrix} \\\\
\mathbf{e}_1 = \begin{bmatrix}
    1 & 0 & 0
\end{bmatrix}^T, \quad
\mathbf{e}_2 = \begin{bmatrix}
    0 & 1 & 0
\end{bmatrix}^T
\end{align}$$




$$
\begin{align}
\min_{t_f, \mathbf{T}_c, \Gamma} \quad & \int_0^{t_f} \Gamma(t)dt \\\\
& \|E\mathbf{r}(t_f) - \mathbf{q}\| \leq \|\mathbf{d}^\*\_{P1} - \mathbf{q}\|
\end{align}
$$

Następujące podstawienie zostało wykonane przez autorów publikacji, aby pozbyć się nieliniowości w dynamice modelu spowodowanych przez \\(\mathbf{T}_c/m\\):

$$
\sigma \triangleq \frac{\Gamma}{m}, \quad \mathbf{u} \triangleq \frac{\mathbf{T}_c}{m}, \quad z \triangleq \ln{m}
$$

Mass loss dynamics

$$
\dot{z} = \frac{\dot{m}(t)}{m(t)} = -\alpha\sigma(t)
$$

$$
\begin{align}
    \|\mathbf{u}(t)\| & \leq \sigma(t), \quad \mathbf{\hat{n}}^T\mathbf{u}(t) \geq \cos{\theta}\sigma(t) \quad \forall t \in [0, t_f] \nonumber \\\\
    \rho_1e^{-z(t)} & \leq \sigma(t) \leq \rho_2e^{-z(t)} \quad \forall t \in [0, t_f] \label{eq:noncvx}
\end{align}
$$

$$
\begin{align}
    \sigma(t) & \geq \rho_1e^{-z_0}\left[ 1 - (z(t) - z_0(t)) + \frac{(z(t) - z_0(t))^2}{2}\right] \quad \forall t \in [0, t_f] \\\\
    \sigma(t) & \leq \rho_2e^{-z_0}[1 - (z(t) - z_0(t))] \quad \forall t \in [0, t_f]
\end{align}
$$

gdzie \\(z_0(t) = \ln{m_0 - \alpha\rho_2 t}\\), a \\(m_0\\) jest masą początkową pojazdu. Warto zauważyć, że z uwagi na obecność zmiennej \\(z(t)^2\\) dolne ograniczenie jest nieliniowe.

$$
\begin{align}
    \min_{T, \mathbf{u}, \sigma} \quad & \int_0^{T} \sigma(t)dt \\\\
    \textrm{p.o.} \quad & \mathbf{x}[k] \in \mathbf{X} \quad \forall k \in [0, N_t] \\\\
    & \mathbf{x}[k] = \begin{bmatrix}
        \mathbf{r}[k] \\\\
        \mathbf{v}[k]
    \end{bmatrix} \\\\
    & \mathbf{\dot{r}}[k] = T \cdot \mathbf{v}[k] \\\\
    & \mathbf{\dot{v}}[k] = T \cdot ( \mathbf{u}[k] + \mathbf{g}) \\\\
    & \dot{z} = - T \cdot \alpha \sigma(\tau) \\\\
    & \rho_1e^{-z_0[k]}\left[ 1 - (z[k] - z_0[k])) + \frac{(z[k] - z_0[k])^2}{2}\right]  - \sigma[k] \leq 0 \\\\
    & \sigma[k] - \rho_2e^{-z_0[k]}[1 - (z[k] - z_0[k])]\leq 0 \\\\
    & z_0[k] = \ln{(m_0 - \alpha\rho_2 t)}, \quad t = kT / N_t \\\\
    & z[0] = \ln{m_0}, \quad z[N_t] \geq \ln(m_0 - m_f), \quad m_0 - m_f > 0 \\\\
    & \mathbf{r}[0] = \mathbf{r}_0, \quad \dot{\mathbf{r}}[0] = \dot{\mathbf{r}}_0  \\\\
    & \mathbf{r}[N_t][0:2] = [0, 0]^T, \enspace \dot{\mathbf{r}}[N_t] = [0, 0, 0]^T \\\\
    & \|\mathbf{u}[k]\| - \sigma[k] \leq 0 \\\\
    & \cos{\theta} \sigma[k] - \mathbf{u}[k][0] \leq 0 \\\\
    & \mathbf{x}[k+1] - \mathbf{x}[k] - \Gamma_f(\mathbf{x}[k], \mathbf{u}[k], T) = \mathbf{0}
\end{align}
$$

Experiment 1.

$$
\begin{align}
    \theta \in & \{\pi, \, \pi/2, \, \pi/4 \} \\\\
    m_f = & 300 \\\\
    m_0 = & 2000 \\\\
    \alpha = & 5 \cdot 10^{-4} \\\\
    T_{max} = & 24000 \\\\
    \rho_1 = & 0.2 \cdot T_{max} \\\\
    \rho_2 = & 0.8 \cdot T_{max} \\\\
    \mathbf{r}_0 & = [2400,\, 450,\, -330]^T \\\\
    \mathbf{v}_0 & = [-10,\, -40,\, 10]^T \\\\
    \mathbf{g} & = [-3.71,\, 0,\, 0]^T 
\end{align}
$$

Experiment 2.

$$
\begin{align}
    \theta & = 120^\circ \\\\
    \gamma_{gs} & = 30^\circ \\\\
    m_f & = 350 \\\\
    \mathbf{r}_0 & = [2400,\, 3400,\, 0]^T \\\\
    \mathbf{v}_0 & = [-40,\, 45,\, 0]^T 
\end{align}
$$

```python
import casadi
import casadi.casadi as cs
import numpy as np
from dataclasses import dataclass


@dataclass
class Solution:
    T: np.float64
    r: np.ndarray
    v: np.ndarray
    z: np.ndarray
    nu: np.ndarray
    sigma: np.ndarray
    nu_mag: np.ndarray
    t_grid: np.ndarray


def run_problem(
    N: int,
    theta: float,
    gamma_gs: float,
    m_fuel: float,
    m_total: float,
    alpha: float,
    rho_1: float,
    rho_2: float,
    v_max: float,
    r0: np.ndarray,
    v0: np.ndarray,
    g: np.ndarray,
):
    opti = casadi.Opti()

    X = opti.variable(7, N + 1)
    r = X[:3, :]
    v = X[3:6, :]
    z = X[6, :]

    # Initial states
    z0 = np.log(m_total)
    opti.subject_to(r[:, 0] == r0)
    opti.subject_to(v[:, 0] == v0)
    opti.subject_to(z[0] == z0)

    for i in range(N):
        opti.set_initial(X[:, i], [*r0, *v0, z0])
    
    opti.set_initial(X[:, N], [0, 0, 0, 0, 0, 0, z0])

    U = opti.variable(4, N)
    nu = U[:3, :]
    sigma = U[3, :]

    T = opti.variable()

    dt = T / N

    def dynamics(x, u):
        return cs.vertcat(
            x[3:6],
            g + u[:3],
            -alpha * u[3],
        )

    cost = 0
    for k in range(N):
        X_next = X[:, k] + dt * dynamics(X[:, k], U[:, k])
        nu_k, sigma_k = U[:3, k], U[3, k]
        cost += dt * sigma_k
        opti.subject_to(X[:, k+1] == X_next)
        opti.subject_to(cs.norm_2(nu_k) <= sigma_k)
        opti.set_initial(nu_k, [1, 0, 0])
        opti.set_initial(sigma_k, 1)

        t = T * k / N
        z0 = cs.log(m_total - alpha * rho_2 * t)
        zk = X_next[6]
        opti.subject_to(rho_1 * cs.exp(-z0) * (1 - (zk - z0) + 0.5 * (zk - z0)**2) <= sigma_k)
        opti.subject_to(sigma_k <= rho_2 * cs.exp(-z0) * (1 - (zk - z0)))
        opti.subject_to(np.cos(theta) * sigma_k <= nu_k[0])
        opti.subject_to(cs.norm_2(X_next[3:6]) <= v_max)

    # Glide slope constraint
    x_f = X[:, N]
    for k in range(N):
        x_k = X[:, k]
        opti.subject_to(
            cs.norm_2(x_k[1:3] - x_f[1:3]) - (x_k[0] - x_f[0]) / np.tan(gamma_gs) <= 0.   
        )
 
    # Stop in 0, 0, 0
    opti.subject_to(X[:6, N] == [0] * 6)

    opti.minimize(cost)
    opti.solver('ipopt')

    # Duration
    opti.subject_to(1.2 <= T)
    opti.subject_to(T <= 70.)
    opti.set_initial(T, 10)

    try:
        sol = opti.solve()
    except RuntimeError as e:
        print(e)

    return Solution(
        T=sol.value(T),
        r=sol.value(r),
        v=sol.value(v),
        z=sol.value(z),
        nu=sol.value(nu),
        sigma=sol.value(sigma),
        nu_mag=sol.value(cs.sqrt(nu[0, :]**2 + nu[1, :]**2 + nu[2, :]**2)),
        t_grid=np.linspace(0, sol.value(T), N + 1)
    )
```