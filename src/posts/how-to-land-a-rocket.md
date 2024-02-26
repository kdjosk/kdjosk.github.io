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

They both are based on the paper titled _"Lossless Convexification of Nonconvex Control
Bound and Pointing Constraints of the Soft
Landing Optimal Control Problem"_. In the first part of this series I'd like to introduce the algorithm and present a simple implementation using the Python API of [CasADi](https://web.casadi.org/), an _"open-source tool for nonlinear optimization and algorithmic differentiation."_. There are two consecutive optimization problems presented in this paper. The first one tries to land the rocket as close to the landing point as possible. The second one tries to improve on the previous solution

$$\begin{align}
    \min_{t_f, \mathbf{T}_c, \Gamma} \quad & \| E\mathbf{r}(t_f) - \mathbf{q}\| \label{eq:p1min} \\\\
    \textrm{p.o.} \quad & \mathbf{x}(t) \in \mathbf{X} \quad \forall t \in [0, t_f] \label{con1} \\\\
    & 0 < \rho_1 \leq \|\mathbf{T}_c\| \leq \rho_2, \quad \hat{\mathbf{n}}^T\mathbf{T}_c(t) \geq \|\mathbf{T}_c\| \cos{\theta} \label{con2} \\\\
    & m(0) = m_0, \quad m(t_f) \geq m_0 - m_f > 0 \label{con3} \\\\
    & \mathbf{r}(0) = \mathbf{r_0}, \enspace \dot{\mathbf{r}}(0) = \dot{\mathbf{r}}_0 \label{con4} \\\\
    & \mathbf{e}^T_1\mathbf{r}(t_f) = 0, \enspace \dot{\mathbf{r}}(tf) = \mathbf{0} \label{con5} \\\\
    & \dot{\mathbf{x}}(t) = \mathbf{A}\mathbf{x}(t) + \mathbf{B}\left(\mathbf{g} + \frac{\mathbf{T}_c(t)}{m}\right) \quad \forall t \in [0, t_f] \label{con6} \\\\
    & \dot{m}(t) = -\alpha \Gamma(t) \label{con7} \\\\
    & \|\mathbf{T}_c(t)\| \leq \Gamma(t), \enspace 0 < \rho_1 \leq \Gamma(t) \leq \rho_2 \label{con8} \\\\
    & \hat{\mathbf{n}}^T\mathbf{T}_c(t) \geq \cos{\theta} \Gamma(t) \label{con9}
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

Definition of the set of possible positions

$$
\mathbf{X} = \{ (\mathbf{r}, \mathbf{\dot{r}}) \in \mathbb{R}^6 \, : \, \|\mathbf{\dot{r}} \| \leq V_{max}, \; \|E(\mathbf{r} - \mathbf{r}(t_f))\| - \mathbf{c}^T(\mathbf{r} - \mathbf{r}(t_f) \leq 0 \}
$$

where \\(\mathbf{c}\\) defines a cone with it's point in \\(\mathbf{r}(t_f)\\), parametrized with the glide slope angle \\(\gamma_{gs}\\)

$$
\mathbf{c} \triangleq \frac{\mathbf{e}\_1}{\tan{\gamma_{gs}}}, \quad \gamma_{gs} \in (0,\, \pi / 2).
$$

$$
\begin{align}
\min_{t_f, \mathbf{T}_c, \Gamma} \quad & \int_0^{t_f} \Gamma(t)dt \\\\
& \|E\mathbf{r}(t_f) - \mathbf{q}\| \leq \|d^\*\_{P1} - \mathbf{q}\|
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

