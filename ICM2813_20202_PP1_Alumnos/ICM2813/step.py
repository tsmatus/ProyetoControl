import numpy as np
from functools import partial
from scipy.integrate import solve_ivp
from ICM2813.Motor import DCMotor, Y

def step_model(model, u, t0, step_size, x0):
    tfinal = t0 + step_size
    Nsamp = 10+1
    tX = np.linspace(t0, tfinal, Nsamp)
    x = solve_ivp(partial(model, u=u), (t0, tfinal), x0, method="BDF", t_eval=tX)

    return Y(x.y.T[-1,:])