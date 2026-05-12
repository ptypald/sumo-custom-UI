import numpy as np
import matplotlib.pyplot as plt

# ---------- Unconstrained Problem ----------
class UnconstrainedOC:
    
    def __init__(self, Xf, Vf, T, x0, v0, N=400):
        self.Xf, self.Vf, self.T = Xf, Vf, T
        self.x0, self.v0, self.N = x0, v0, N

    def closed_form(self, x0, v0, Xf, Vf, T, t):
        dv = Vf - v0
        dx = Xf - x0 - v0*T
        
        a = 6*dv/T**2 - 12*dx/T**3
        b = -2*dv/T + 6*dx/T**2
        c = v0
        d = x0

        v = c + 0.5*a*t**2 + b*t
        x = d + c*t + (a/6)*t**3 + 0.5*b*t**2
        u = a*t + b
        return x, v, u
    
    def solve(self):
        t = np.linspace(0.0, self.T, self.N+1)
        x, v, u = self.closed_form(self.x0, self.v0, self.Xf, self.Vf, self.T, t)
        return t, x, v, u

# ---------- Constrained Problem ----------
class ConstrainedOC:

    def __init__(self, x0, v0, xe, ve, T, tr, xr, N=400):
        self.x0, self.v0 = x0, v0
        self.xe, self.ve = xe, ve
        self.T, self.tr, self.xr = T, tr, xr
        self.N = N

    def coeffs_constrained(self, x0, v0, xe, ve, T, tr, xr):
        # Denominators
        D  = (T - tr) * (T + 3*tr)
        D2 = (T - tr)**3 * (T + 3*tr)

        # Analytic coefficients (a1,b1,c1,d1,a2,b2,c2,d2)
        a1 = 6*( T**2*tr*v0 + T**2*(x0 - xr) - T*tr**2*ve
                + 2*T*tr*(x0 - xr) - tr**3*(v0 - ve) - 3*tr**2*(x0 - xe) ) / (tr**3 * D)

        a2 = 3*( T**2*(v0 + ve) - 2*T*(tr*v0 + xe - xr) + tr**2*(v0 - ve) ) / D2

        b2 = 2*( 2*T*v0 + T*ve - 2*tr*v0 - tr*ve - 3*xe + 3*xr ) / D

        c1, d1 = v0, x0
        c2 = ( T**2*v0 - 2*T*tr*(v0 + ve) + tr**2*(v0 + 2*ve) + 6*tr*(xe - xr) ) / D
        d2 = xr

        b1 = a1*tr + b2

        return a1, b1, c1, d1, a2, b2, c2, d2

    def solve(self):
        x0, v0, xe, ve, T, tr, xr = self.x0, self.v0, self.xe, self.ve, self.T, self.tr, self.xr
        a1, b1, c1, d1, a2, b2, c2, d2 = self.coeffs_constrained(x0, v0, xe, ve, T, tr, xr)

        t = np.linspace(0.0, T, self.N+1)
        m1 = t <= tr
        m2 = t >= tr

        t1 = t[m1]
        u1 = a1*t1 - b1
        v1 = a1*t1**2 - b1*t1 + c1
        x1 = (1/3)*a1*t1**3 - 0.5*b1*t1**2 + c1*t1 + d1

        t2 = t[m2]; s2 = t2 - tr
        u2 = a2*s2 - b2
        v2 = a2*s2**2 - b2*s2 + c2
        x2 = (1/3)*a2*s2**3 - 0.5*b2*s2**2 + c2*s2 + d2

        x = np.concatenate([x1, x2[1:]]) if x1.size and x2.size else np.concatenate([x1, x2])
        v = np.concatenate([v1, v2[1:]]) if v1.size and v2.size else np.concatenate([v1, v2])
        u = np.concatenate([u1, u2[1:]]) if u1.size and u2.size else np.concatenate([u1, u2])
        t_out = np.concatenate([t1, t2[1:]]) if t1.size and t2.size else np.concatenate([t1, t2])

        return t_out, x, v, u


def main():

    # Unconstrained example
    x0, v0 = 0.0, 0.0
    Xf, Vf, T = 200.0, 0.0, 20.0
    uc = UnconstrainedOC(Xf, Vf, T, x0, v0, N=600)
    t_u, x_u, v_u, u_u = uc.solve()

    # Constrained example
    tr, xr = 12.0, 100.0
    cc = ConstrainedOC(x0, v0, Xf, Vf, T, tr, xr, N=600)
    t_c, x_c, v_c, u_c = cc.solve()

    # Plots
    plt.figure()
    plt.plot(t_u, x_u, label="x unconstrained")
    plt.plot(t_c, x_c, label="x constrained")
    plt.hlines(y=xr, xmin=0.0, xmax=tr, colors='red', linewidth=2, label='Red Phase')
    plt.hlines(y=xr, xmin=tr, xmax=T, colors='green', linewidth=2, label='Green Phase')
    plt.axvline(tr, ls=":", color='k', alpha=0.7, label=f"t_r={tr:g}")
    plt.xlabel("Time [s]"); plt.ylabel("Position [m]"); plt.title("Position"); plt.legend()

    plt.figure()
    plt.plot(t_u, v_u, label="v unconstrained")
    plt.plot(t_c, v_c, label="v constrained")
    plt.xlabel("Time [s]"); plt.ylabel("Speed [m/s]"); plt.title("Speed"); plt.legend()

    plt.figure()
    plt.plot(t_u, u_u, label="u unconstrained")
    plt.plot(t_c, u_c, label="u constrained")
    plt.xlabel("Time [s]"); plt.ylabel("Acceleration [m/s²]"); plt.title("Control"); plt.legend()

    plt.show()

    plt.show()

if __name__ == "__main__":
    main()
