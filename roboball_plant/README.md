# Friction Models Description

The `joint_modifiers.py` file contains two classes that are used to model various types of joint frictions. All of the functions use $v$ ad joint velocity. 

the first one a `StictionModel()` class is instantiated with a list of two parameters. The static friction coeficient ($\mu_s$), dynamic friction coeficient ($\mu_d$) and the viscous damping coefficient ($c$). 

The resulting resistive torque is calulated by the following equation

> $$ \tau_{friction} = \begin{cases}  \mu_s sign(v),  & \text{if } v < tol \\
                      -\mu_d sign(v) - cv & \text{otherwise} \end{cases}$$

the second is  `StictionModel_majd()` that follows a similar structure to the first. It is instantiated with a list of 5 parameters used in the equation.

The resulting resistive torque is calulated by the following equation [1]:

> $$\tau_{friction} = f_\omega v [f_c + \sigma e^{-|v/\omega_c|} - (\sigma+f_c) e^{-|n v /\omega_c|}]sign(v)$$


The parameters we found that made a good fit are summarized in the table below.

| -  | Stiction | Majd |
| ----------- | ----------- | ----------- |
| Parameter List | [ $\mu_s$, $\mu_d$, $c$] | [ $f_w$, $f_c$, $\sigma$, $\omega_c$, $n$]|
| Values | [1, 0.8, 0.22] |  [0.23, 0.7, 10, 1, 1] |


# References 

[1] V. J. Majd and M. A. Simaan, "A continuous friction model for servo systems with stiction," Proceedings of International Conference on Control Applications, Albany, NY, USA, 1995, pp. 296-301, doi: 10.1109/CCA.1995.555719.
